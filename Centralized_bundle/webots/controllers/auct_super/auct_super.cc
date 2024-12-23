/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * file:        auct_super.cc
 * author:      
 * description: Supervisor for market-based task allocation (DIS lab05)
 *
 * $Revision$	February 2016 by Florian Maushart
 * $Date$
 * $Author$   Last update 2024 by Wanting Jin
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include <assert.h>
#include <bitset>
#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <map>

#include <vector>
#include <memory>
#include <time.h>       /* time_t, struct tm, difftime, time, mktime */
#include <algorithm>
using namespace std;

#include "Point2d.h"
#include "message.h"

#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/supervisor.h>
#include <webots/robot.h>

#define DBG(x) printf x
#define RAND ((float) rand()/RAND_MAX)

#define MAX_ROBOTS 5
#define MAX_EVENTS 10

#define STEP_SIZE 64            // simulation step size
#define AUCTION_TIMEOUT 128    // number of steps after which an auction stops

#define EVENT_RANGE (0.1)       // distance within which a robot must come to do event
#define EVENT_TIMEOUT (5000)   // ticks until an event auction runs out
#define EVENT_GENERATION_DELAY (1000) // average time between events ms (expo distribution)

#define GPS_INTERVAL (500)

// Parameters that can be changed
#define NUM_ROBOTS 5                 // Change this also in the epuck_crown.c!
#define NUM_ACTIVE_EVENTS 10         // number of active events
#define TOTAL_EVENTS_TO_HANDLE  50   // Events after which simulation stops or...
#define MAX_RUNTIME (3*60*1000)      // ...total runtime after which simulation stops

#define WALL_MARGIN 0.01  // Collision margin from the wall
#define ARENA_WIDTH 1.0  // Arena width in meters
#define ARENA_HEIGHT 1.0 // Arena height in meters

struct RobotState {
  bool state;         // 0: idle or going to goal   |   1: handling task
  int counter; 
};

WbNodeRef g_event_nodes[MAX_EVENTS];
vector<WbNodeRef> g_event_nodes_free;
map<uint16_t, RobotState> robotStates;



double gauss(void) 
{
  double x1, x2, w;
  do {
      x1 = 2.0 * RAND - 1.0;
      x2 = 2.0 * RAND - 1.0;
      w = x1*x1 + x2*x2;
  } while (w >= 1.0);

  w = sqrt((-2.0 * log(w))/w);
  return(x1*w);
}

double rand_coord() {
  return -0.45 + 0.9 * RAND;
}

double expovariate(double mu) {
  double uniform = RAND;
  while (uniform < 1e-7) uniform = RAND;
  return -log(uniform) * mu;
}

uint16_t set_counter(Event_type event_type, uint16_t robot_id){
  if (robot_id < 2){              // robot of type A
    if (event_type == A){
      return 3000;    //[ms]
    }
    else{
      return 5000;
    }
  }else{                          // robot of type B
    if (event_type == A){
      return 9000;
    }
    else{
      return 1000;
    }
  }
}

void changeCylinderColor(WbNodeRef node, double r, double g, double b) {
  WbFieldRef childrenField = wb_supervisor_node_get_field(node, "children");
    
  WbNodeRef shapeNode = wb_supervisor_field_get_mf_node(childrenField, 0); // Get the Shape node
  WbFieldRef appearanceField = wb_supervisor_node_get_field(shapeNode, "appearance");
  

  WbNodeRef appearanceNode = wb_supervisor_field_get_sf_node(appearanceField); // Get Appearance node
  WbFieldRef materialField = wb_supervisor_node_get_field(appearanceNode, "material");
  

  WbNodeRef materialNode = wb_supervisor_field_get_sf_node(materialField); // Get Material node
  WbFieldRef diffuseColorField = wb_supervisor_node_get_field(materialNode, "diffuseColor");

  // Set the new color
  const double newColor[3] = {r, g, b};
  wb_supervisor_field_set_sf_color(diffuseColorField, newColor);
}

// Event class
class Event {

// Public variables
public:
  uint16_t id_;          //event id
  Point2d pos_;          //event pos
  WbNodeRef node_;       //event node ref
  uint16_t assigned_to_; //id of the robot that will handle this event
  Event_type type_;

  // Auction data
  uint64_t t_announced_;        //time at which event was announced to robots
  bitset<NUM_ROBOTS> bids_in_;
  uint16_t best_bidder_;        //id of the robot that had the best bid so far
  double best_bid_;             //value of the best bid (lower is better)
  uint64_t t_done_;             //time at which the assigned robot reached the event
  int bidder_index;             //index at which the bidder will put event in tasklist

// Public functions
public:
  //Event creation
  Event(uint16_t id) : id_(id), pos_(rand_coord(), rand_coord()),
    assigned_to_(-1), t_announced_(-1), best_bidder_(-1), best_bid_(0.0), t_done_(-1)
  {
    node_ = g_event_nodes_free.back();  // Get the node from the free event list
    g_event_nodes_free.pop_back();      // and remove it from the free event list

    // draw event type: 1/3 -> A  |  2/3 -> B
    if (RAND <= 0.3){
      type_ = A;
      changeCylinderColor(node_, 1, 0, 0);
    }else{
      type_ = B;
      changeCylinderColor(node_, 0, 0, 1);
    }
    
    double event_node_pos[3];           // Place event in arena
    event_node_pos[0] = pos_.x;
    event_node_pos[1] = pos_.y;
    event_node_pos[2] = .01;
    wb_supervisor_field_set_sf_vec3f(
      wb_supervisor_node_get_field(node_,"translation"),
      event_node_pos);

  }

  bool is_assigned() const { return assigned_to_ != (uint16_t) -1; }
  bool was_announced() const { return t_announced_ != (uint64_t) -1; }
  bool has_bids() const { return best_bidder_ != (uint16_t) -1; }
  bool is_done() const { return t_done_ != (uint64_t) -1; }

  // Check if event can be assigned
  void updateAuction(uint16_t bidder, double bid, int index) {
    if (bid >= 0.0 && (!has_bids() || bid < best_bid_)) {
      best_bidder_ = bidder;
      best_bid_ = bid;
      bidder_index = index;  
    }
    bids_in_.set(bidder);
    if (bids_in_.all()) assigned_to_ = best_bidder_;
  }

  void restartAuction() {
    assigned_to_ = -1;
    t_announced_ = -1;
    bids_in_.reset();
    best_bidder_ = -1;
    best_bid_ = 0.0;
    t_done_ = -1;
  }

  void markDone(uint64_t clk) {
    t_done_ = clk;
    double event_node_pos[3] = {-5,-5,0.1};
    wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(node_,"translation"),
                                     event_node_pos);
    g_event_nodes_free.push_back(node_);
  }
};

class Bundle {

// Public variables
public:
  vector<Event*> event_list;
  uint16_t assigned_to_; //id of the robot that will handle this event

  // Auction data
  uint64_t t_announced_;        //time at which event was announced to robots
  bitset<NUM_ROBOTS> bids_in_;
  uint16_t best_bidder_;        //id of the robot that had the best bid so far
  double best_bid_;             //value of the best bid (lower is better)
  uint64_t t_done_;             //time at which the assigned robot completed the bundle
  int bidder_arrangement_[3];      //arrangement that will be used by the bidder to complete the tasks.
// Public functions
public:
  // Bundle creation
  Bundle() : assigned_to_(-1), t_announced_(-1), best_bidder_(-1), best_bid_(0.0), t_done_(-1) {
    // Constructor body
  }

  bool is_assigned() const { return assigned_to_ != (uint16_t) -1; }
  bool was_announced() const { return t_announced_ != (uint64_t) -1; }
  bool has_bids() const { return best_bidder_ != (uint16_t) -1; }
  bool is_done() const { return t_done_ != (uint64_t) -1; }

  void reset(){

    event_list.clear();

    assigned_to_ = -1;
    t_announced_ = -1;
    bids_in_ = 0;
    best_bidder_ = -1;
    best_bid_ = -1;
    t_done_ = -1;

    for (int i = 0; i < 3; i++){
      bidder_arrangement_[i] = 0;
    }
  }

  // Check if bundle can be assigned // RIGHT NOW IT WAITS FOR ALL ROBOT TO BID
  void updateAuction(uint16_t bidder, double bid, int arrangement[3]) {
    if (bid >= 0.0 && (!has_bids() || bid < best_bid_)) {
      best_bidder_ = bidder;
      best_bid_ = bid;
      for (int i = 0; i < 3; ++i) {
            bidder_arrangement_[i] = arrangement[i];
      }
    }
    bids_in_.set(bidder);
    if (bids_in_.all()) assigned_to_ = best_bidder_;    //wait for all bidders
  }

  void restartAuction() {
    assigned_to_ = -1;
    t_announced_ = -1;
    bids_in_.reset();
    best_bidder_ = -1;
    best_bid_ = 0.0;
    t_done_ = -1;
  }

  void markDone(uint64_t clk) {
    t_done_ = clk;
  }

};

Bundle bundle;

bool auction = false;

// Supervisor class
class Supervisor {

//Private variables
private:
  uint64_t clock_;

  uint16_t next_event_id_;
  vector<unique_ptr<Event> > events_;
  uint16_t num_active_events_;
  uint64_t t_next_event_;
  uint64_t t_next_gps_tick_;

  uint16_t num_events_handled_; // total number of events handled
  double stat_total_distance_;  // total distance traveled
  double stat_robot_prev_pos_[NUM_ROBOTS][2];

  uint64_t total_active_time_[NUM_ROBOTS] = {0}; // Total active time for each robot
  uint64_t total_collisions_ = 0;                // Total number of collisions
  uint64_t total_simulation_time_ = 0;           // Total simulation time
  uint64_t stat_total_time_handeling_tasks_ = 0;

  std::vector<std::pair<int, int>> active_collisions_; // List of active collisions

  WbNodeRef robots_[NUM_ROBOTS];
  WbDeviceTag emitter_;
  WbDeviceTag receivers_[NUM_ROBOTS];

  typedef vector<pair<Event*, message_event_state_t>> event_queue_t;
  typedef vector<pair<Bundle, message_event_state_t>> bundle_queue_t;

// Private functions
private:

  bool isRobotAssignedToEvent(uint16_t robot_id) {
    for (const auto& event : events_) {
        if (event->is_assigned() && !event->is_done() && event->assigned_to_ == robot_id) {
            return true;
        }
    }
    return false;
  }

  void addEvent() {
    events_.push_back(unique_ptr<Event>(new Event{next_event_id_++})); // add to list
    assert(num_active_events_ < NUM_ACTIVE_EVENTS); // check max. active events not reached
    num_active_events_++;
    t_next_event_ = clock_ + expovariate(EVENT_GENERATION_DELAY);
  }

  // Init robot and get robot_ids and receivers
  void linkRobot(uint16_t id) {
    const char kRobotNameFormat[] = "e-puck%d";
    const char kReceiverNameFormat[] = "rec%d";
    char node_name[16];

    // Get the robot node's handle
    sprintf(node_name, kRobotNameFormat, id);
    robots_[id] = wb_supervisor_node_get_from_def(node_name);
    if (!robots_[id]) {
      DBG(("Missing node for robot #%d\n", id));
      exit(1);
    }

    // Get the respective receiver
    sprintf(node_name, kReceiverNameFormat, id); 
    receivers_[id] = wb_robot_get_device(node_name);
    if (!receivers_[id]) {
      DBG(("Missing receiver for robot #%d\n", id));
      exit(1);
    }
    wb_receiver_enable(receivers_[id], 2); //32
    wb_receiver_set_channel(receivers_[id], id+1);
  }

  // Assemble a new message to be sent to robots
  void buildMessage(uint16_t robot_id, const Event* event,
      message_event_state_t event_state, message_t* msg) {
    WbFieldRef f_rot = wb_supervisor_node_get_field(robots_[robot_id],
                                                    "rotation");
    const double *pos = getRobotPos(robot_id);
    const double *rot = wb_supervisor_field_get_sf_rotation(f_rot);

    msg->robot_id = robot_id;
    msg->robot_x = pos[0]; // no gps noise used here
    msg->robot_y = pos[1]; // no gps noise used here
    double heading = -rot[2] *rot[3]; // no gps noise used here
    msg->heading = heading > 2*M_PI ? heading - 2*M_PI : heading;
    msg->event_state = event_state;
    msg->event_id = -1;
    msg->event_x = 0.0;
    msg->event_y = 0.0;

    if (event) {
      assert(event_state != MSG_EVENT_INVALID && 
             event_state != MSG_EVENT_GPS_ONLY);
      msg->event_id = event->id_;
      msg->event_x = event->pos_.x;
      msg->event_y = event->pos_.y;
      msg->event_index = event->bidder_index;
      msg->event_type = event->type_;
    }
  }

    // Assemble a new message to be sent to robots
  void buildAuctionMessage(uint16_t robot_id, message_event_state_t bundle_state, message_t* msg) {

    msg->robot_id = robot_id;
    msg->event_state = bundle_state;
    
    msg->a1_x = bundle.event_list[0]->pos_.x;
    msg->a1_y = bundle.event_list[0]->pos_.y;
    msg->a1_type = bundle.event_list[0]->type_;
    msg->a2_x = bundle.event_list[1]->pos_.x;
    msg->a2_y = bundle.event_list[1]->pos_.y;
    msg->a2_type = bundle.event_list[1]->type_;
    msg->a3_x = bundle.event_list[2]->pos_.x;
    msg->a3_y = bundle.event_list[2]->pos_.y;
    msg->a3_type = bundle.event_list[2]->type_;

  }

  const double* getRobotPos(uint16_t robot_id) {
    WbFieldRef f_pos = wb_supervisor_node_get_field(robots_[robot_id],
      "translation");
    return wb_supervisor_field_get_sf_vec3f(f_pos);
  }

  void setRobotPos(uint16_t robot_id, double x, double y) {
    WbFieldRef f_pos = wb_supervisor_node_get_field(robots_[robot_id],
      "translation");
    double pos[3] = {x, y, 0};
    return wb_supervisor_field_set_sf_vec3f(f_pos, pos);
  }

  // Marks one event as done, if one of the robots is within the range
  void markEventsDone(event_queue_t& event_queue) {
    for (auto& event : events_) {  //iterate through all the events of the supervisor (keep in memory even old events)
      if (!event->is_assigned() || event->is_done())
        continue;
      
      const double *robot_pos = getRobotPos(event->assigned_to_);
      Point2d robot_pos_pt(robot_pos[0], robot_pos[1]);
      double dist = event->pos_.Distance(robot_pos_pt);

      if (dist <= EVENT_RANGE) {
        //printf("[REACHED] Robot %d reached event %d which is of type %s\n", event->assigned_to_,
        //        event->id_, (event->type_ == A) ? "A" : "B");

        if (robotStates[event->assigned_to_].state == 0){     // robot arrives in range but is not handling the task yet 
          robotStates[event->assigned_to_].state = 1;         // starts handling the task  
          robotStates[event->assigned_to_].counter = set_counter(event->type_, event->assigned_to_);
          event_queue.emplace_back(event.get(), MSG_EVENT_BEING_HANDLED);
        }
        else if (robotStates[event->assigned_to_].state == 1 && robotStates[event->assigned_to_].counter > 0){ // the robot is handling the task but isn't finish yet
          robotStates[event->assigned_to_].counter -= STEP_SIZE;
        }
        else{     // the robot finished the task
          robotStates[event->assigned_to_].state = 0;
          robotStates[event->assigned_to_].counter = 0;

          num_events_handled_++;      // increase the number of event that were handled
          event->markDone(clock_);    // Timestamp the completion of the event
          num_active_events_--;       // decrease the number of active event
          event_queue.emplace_back(event.get(), MSG_EVENT_DONE);
          addEvent();
        }    
      }
    }
  }

  void handleAuctionBundle(bundle_queue_t& bundle_queue, event_queue_t& event_queue) {
    // For each unassigned event
    for (auto& event : events_) {
      if (event->is_assigned()){
        continue;
      }
      else if (bundle.event_list.size() < 3 && find(bundle.event_list.begin(), bundle.event_list.end(), event.get()) == bundle.event_list.end()){
        // if bundle is not complete and the current event is not in the list
        bundle.event_list.push_back(event.get());
        printf("...creating bundle...\n");
      }
    }
      
    if (bundle.event_list.size() >= 3 && !auction){
      // if the bundle is complete and no auction is taking place, announce the bundle for an auction
      printf("Bundle is complete ! Starting auction !\n");
      bundle.t_announced_ = clock_;
      auction = true;
      bundle_queue.emplace_back(bundle, MSG_BUNDLE_NEW);
      printf("[BUNDLE] An bundle with task %d, %d and %d was announced\n", bundle.event_list[0]->id_, bundle.event_list[1]->id_, bundle.event_list[2]->id_);
    } 
    else if (bundle.event_list.size() >= 3 && clock_ - bundle.t_announced_ > EVENT_TIMEOUT){
      // if not all robot have bid something and that EVENT_TIMEOUT time has passed out, give the bundle to the highest bidder
      printf("bundle has bids %s\n", bundle.has_bids() ? "true" : "false");
      if (bundle.has_bids()){
        bundle.assigned_to_ = bundle.best_bidder_;
        bundle_queue.emplace_back(bundle, MSG_BUNDLE_WON);
        auction = false;

        // assign the events in the best order
        for (int m = 0; m < 3; m++) {
            int index = bundle.bidder_arrangement_[m];
            Event* event = bundle.event_list[index];
            event->assigned_to_ = bundle.assigned_to_;
            event->bidder_index = m;
            event_queue.emplace_back(event, MSG_EVENT_WON);
        }

        printf("[AUCTION] Robot %d won the bundle\n", bundle.assigned_to_);
        bundle.reset();

      } else {
          // (reannounced the bundle in next iteration)
          bundle.restartAuction();
          auction = false;
      }
    }
  }

void updateMetricsAndDistance() {
  // Increment simulation time
  total_simulation_time_ += STEP_SIZE;

  for (int i = 0; i < NUM_ROBOTS; ++i) {
    // Get current robot position
    const double *robot_pos = getRobotPos(i);

    // Calculate distance traveled
    double delta[2] = {
      robot_pos[0] - stat_robot_prev_pos_[i][0],
      robot_pos[1] - stat_robot_prev_pos_[i][1]
    };
    double distance = sqrt(delta[0]*delta[0] + delta[1]*delta[1]);
    stat_total_distance_ += distance;

    // Update previous position
    stat_robot_prev_pos_[i][0] = robot_pos[0];
    stat_robot_prev_pos_[i][1] = robot_pos[1];

    // Update time metrics
    if (robotStates[i].state == 1) { // Robot is handling a task
        total_active_time_[i] += STEP_SIZE;
        stat_total_time_handeling_tasks_ += STEP_SIZE;
    } else if (isRobotAssignedToEvent(i)) {
        if (distance > 0.0001) { // Robot is moving towards an event
            total_active_time_[i] += STEP_SIZE;
        }
    }


    // Check for collision with walls
    if (robot_pos[0] > ARENA_WIDTH - WALL_MARGIN || robot_pos[1] > ARENA_HEIGHT - WALL_MARGIN) {
      total_collisions_++;
    }
  }

  std::vector<std::pair<int, int>> new_collisions;
  
  // Collision detection logic
  for (int i = 0; i < NUM_ROBOTS; ++i) {
    for (int j = i + 1; j < NUM_ROBOTS; ++j) {
      const double *pos1 = getRobotPos(i);
      const double *pos2 = getRobotPos(j);

      double distance = sqrt(pow(pos1[0] - pos2[0], 2) + pow(pos1[1] - pos2[1], 2));
      if (distance < 0.081) { // Collision detected
        // Add to new collisions if not already present
        if (std::find(active_collisions_.begin(), active_collisions_.end(),
                      std::make_pair(i, j)) == active_collisions_.end()) {
          new_collisions.push_back(std::make_pair(i, j));
        }
      }
    }
  }
  // Count new collisions
  for (const auto &collision : new_collisions) {
    active_collisions_.push_back(collision);
    total_collisions_++;
    //printf("[COLLISION] Robots %d and %d collided\n", collision.first, collision.second);
  }

  // Remove resolved collisions
  active_collisions_.erase(
      std::remove_if(active_collisions_.begin(), active_collisions_.end(),
                     [&](const std::pair<int, int> &collision) {
                       const double *pos1 = getRobotPos(collision.first);
                       const double *pos2 = getRobotPos(collision.second);
                       double distance =
                           sqrt(pow(pos1[0] - pos2[0], 2) +
                                pow(pos1[1] - pos2[1], 2));
                       return distance >= EVENT_RANGE;
                     }),
      active_collisions_.end());
}

// Public fucntions
public:
  Supervisor() : events_(MAX_EVENTS){}
  
  // Reset robots & events
  void reset() {
    clock_ = 0;

    // initialize the robot states
    for (int i=0;i<NUM_ROBOTS;i++) {
      robotStates[i].state = 0;
      robotStates[i].counter = 0;

      printf("robot %d has state %d and counter %d\n", i, robotStates[i].state, robotStates[i].counter);
    }

    // initialize & link events
    next_event_id_ = 0;
    events_.clear();
    num_active_events_ = 0;
    t_next_event_ = 0; // invalid state
    auction = NULL;
    t_next_gps_tick_ = 0;

    num_events_handled_ = 0;
    stat_total_distance_ = 0.0;

    // add the first few events
    for (int i=0; i<NUM_ACTIVE_EVENTS; ++i) {
      addEvent();
    }

    // link & initialize robots
    for (int i=0;i<NUM_ROBOTS;i++) {
      linkRobot(i);

      double pos[2] = {rand_coord(), rand_coord()};
      setRobotPos(i, pos[0], pos[1]);
      stat_robot_prev_pos_[i][0] = pos[0];
      stat_robot_prev_pos_[i][1] = pos[1];
    }

    // initialize the emitter
    emitter_ = wb_robot_get_device("sup_emitter");
    if (!emitter_) {
      DBG(("Missing supervisor emitter!\n"));
      exit(1);
    }
  }

  //Do a step
  bool step(uint64_t step_size) {
    
    clock_ += step_size;

    // Events that have been assigned/done
    event_queue_t event_queue;
    // bundle that will be announced
    bundle_queue_t bundle_queue;

    markEventsDone(event_queue);

    handleAuctionBundle(bundle_queue, event_queue);
 
    // Send and receive messages
    bid_t* pbid; // inbound
    for (int i=0;i<NUM_ROBOTS;i++) {
      //printf("YOOOOOOOOOOOOOOO\n");
      // Check if we're receiving data
      if (wb_receiver_get_queue_length(receivers_[i]) > 0) {
        assert(wb_receiver_get_queue_length(receivers_[i]) > 0);
        assert(wb_receiver_get_data_size(receivers_[i]) == sizeof(bid_t));
        
        pbid = (bid_t*) wb_receiver_get_data(receivers_[i]); 
        assert(pbid->robot_id == i);

        printf("[AUCTION] robot %d placed a bid of %f on the bundle\n ", pbid->robot_id, pbid->value);

        bundle.updateAuction(pbid->robot_id, pbid->value, pbid->arrangement);

        if (bundle.is_assigned()) {
          bundle_queue.emplace_back(bundle, MSG_BUNDLE_WON);
          auction = false;
          
          // send the tasks in the right order the tasks to the winner
          for (int m = 0; m < 3; m++) {
              int index = bundle.bidder_arrangement_[m];
              Event* event = bundle.event_list[index];
              event->assigned_to_ = bundle.assigned_to_;
              event->bidder_index = m;
              event_queue.emplace_back(event, MSG_EVENT_WON);
          }

          printf("[AUCTION] robot %d won the bundle\n", bundle.assigned_to_);

          bundle.reset();
        }

        wb_receiver_next_packet(receivers_[i]);
      }
    }

    // outbound
    message_t msg;
    bool is_gps_tick = false;

    if (clock_ >= t_next_gps_tick_) {
      is_gps_tick = true;
      t_next_gps_tick_ = clock_ + GPS_INTERVAL;
    }

    for (int i=0;i<NUM_ROBOTS;i++) {
      // Send updates to the robot
      while (wb_emitter_get_channel(emitter_) != i+1)
      wb_emitter_set_channel(emitter_, i+1);
      
      if (is_gps_tick) {
        buildMessage(i, NULL, MSG_EVENT_GPS_ONLY, &msg);
//        printf("sending message %d , %d \n",msg.event_id,msg.robot_id);
        while (wb_emitter_get_channel(emitter_) != i+1)
            wb_emitter_set_channel(emitter_, i+1);        
        wb_emitter_send(emitter_, &msg, sizeof(message_t));
      }

      for (const auto& e_es_tuple : event_queue) {
        const Event* event = e_es_tuple.first;
        const message_event_state_t event_state = e_es_tuple.second;
        if (event->is_assigned() && event->assigned_to_ != i) continue;
        // if the event is not assigned, send a message about anouncing and stuff
        // if the event is assigned and the robot is assigned to, is the good one, send the message that it was won  

        buildMessage(i, event, event_state, &msg);
        while (wb_emitter_get_channel(emitter_) != i+1)
              wb_emitter_set_channel(emitter_, i+1);        
//        printf("> Sent message to robot %d // event_state=%d\n", i, event_state);
//        printf("sending message event %d , robot %d , emitter %d, channel %d\n",msg.event_id,msg.robot_id,emitter_,      wb_emitter_get_channel(emitter_));
        
        wb_emitter_send(emitter_, &msg, sizeof(message_t));
      }

      for (const auto& b_es_tuple : bundle_queue) {
        const message_event_state_t bundle_state = b_es_tuple.second;
        if (bundle.is_assigned() && bundle.assigned_to_ != i) continue;

        buildAuctionMessage(i, bundle_state, &msg);
        while (wb_emitter_get_channel(emitter_) != i+1)
              wb_emitter_set_channel(emitter_, i+1);        
//        printf("> Sent message to robot %d // event_state=%d\n", i, event_state);
//        printf("sending message event %d , robot %d , emitter %d, channel %d\n",msg.event_id,msg.robot_id,emitter_,      wb_emitter_get_channel(emitter_));
        
        wb_emitter_send(emitter_, &msg, sizeof(message_t));
      }
    }

    // Keep track of distance travelled by all robots
    updateMetricsAndDistance(); 

    // Time to end the experiment?
    if (MAX_RUNTIME > 0 && clock_ >= MAX_RUNTIME) {
      printf("\n");
      printf("===== End of Simulation: Key metrics ======\n");
      printf("\n");
      for(int i=0;i<NUM_ROBOTS;i++){
          buildMessage(i, NULL, MSG_QUIT, &msg);
          wb_emitter_set_channel(emitter_, i+1);
          wb_emitter_send(emitter_, &msg, sizeof(message_t));

          double active_percentage = (double)total_active_time_[i] / total_simulation_time_ * 100.0;
          printf("Robot %d: Average activation time = %.2f%%\n", i, active_percentage);
      }
      double clock_s = ((double) clock_) / 1000.0;
      double ehr = ((double) num_events_handled_) / clock_s;
      double total_time_handeling_task = stat_total_time_handeling_tasks_;
      printf("Total collisions = %llu\n", total_collisions_);
      printf("Total distance travelled = %f\n", stat_total_distance_);
      printf("Total time handeling task = %f\n", total_time_handeling_task/1000);
      printf("Handled %d events in %d seconds, events handled per second = %.2f\n",
             num_events_handled_, (int) clock_ / 1000, ehr);
      
      return false;
    } 
    else { return true;} //continue
  } // << step() <<
};

//Links up all the nodes we are interested in.
//Gets called by webots at robot_live(reset)
void link_event_nodes() {
  const char kEventNameFormat[] = "e%d";
  char node_name[16];
  
  for (int i=0; i<MAX_EVENTS; ++i) {
    sprintf(node_name, kEventNameFormat, i);
    g_event_nodes[i] = wb_supervisor_node_get_from_def(node_name);
    g_event_nodes_free.push_back(g_event_nodes[i]);
  }
}

// MAIN LOOP (does steps)
int main(void) 
{
  Supervisor supervisor{};

  // initialization
  wb_robot_init();
  link_event_nodes();
  wb_robot_step(STEP_SIZE);

  srand(time(NULL));
  supervisor.reset();

  // start the controller
  printf("Starting main loop...\n");
  while (wb_robot_step(STEP_SIZE) != -1)
  {
    if (!supervisor.step(STEP_SIZE)) break; //break at return = false
  }
  //wb_supervisor_simulation_reset_physics();
  wb_robot_cleanup();
  exit(0);
  return 0;

}