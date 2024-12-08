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
#include <cstring>
#include <set>

#include <vector>
#include <memory>
#include <time.h>       /* time_t, struct tm, difftime, time, mktime */

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
#define RX_PERIOD 2
#define AUCTION_TIMEOUT 1000    // number of steps after which an auction stops

#define EVENT_RANGE (0.1)      // distance within which a robot must come to do event
#define EVENT_TIMEOUT (10000)   // ticks until an event auction runs out
#define EVENT_GENERATION_DELAY (1000) // average time between events ms (expo distribution)

#define GPS_INTERVAL (500)

// Parameters that can be changed
#define NUM_ROBOTS 5                 // Change this also in the epuck_crown.c!
#define NUM_ACTIVE_EVENTS 10           // number of active events
#define TOTAL_EVENTS_TO_HANDLE  50   // Events after which simulation stops or...
#define MAX_RUNTIME (3*60*1000)      // ...total runtime after which simulation stops
//

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
  // return -1.0 + 2.0*RAND;
  return -0.45 + 0.9 * RAND;
}

double expovariate(double mu) {
  double uniform = RAND;
  while (uniform < 1e-7) uniform = RAND;
  return -log(uniform) * mu;
}

uint16_t set_counter(Event_type event_type, uint16_t robot_id){
  if (robot_id < 2){      // robot of type A
    if (event_type == A){
      return 3000;    //[ms]
    }
    else{
      return 5000;
    }
  }else{
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
  Event_type type_;
  int finished_;
  int processed_ = 0;

// Public functions
public:
  //Event creation
  Event(uint16_t id) : id_(id), pos_(rand_coord(), rand_coord()), finished_(0)
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

  bool is_done() const { return finished_ != 0; }

  void markDone(uint64_t clk) {
    double event_node_pos[3] = {-5,-5,0.1};
    wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(node_,"translation"),
                                     event_node_pos);
    g_event_nodes_free.push_back(node_);
  }
};

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
  int events_marked = 0;

  uint16_t num_events_handled_; // total number of events handled
  double stat_total_distance_;  // total distance traveled
  double stat_robot_prev_pos_[NUM_ROBOTS][2];

  WbNodeRef robots_[NUM_ROBOTS];
  WbDeviceTag emitter_;
  WbDeviceTag receivers_[NUM_ROBOTS];
  WbDeviceTag super_receiver_;

  typedef vector<pair<Event*, message_event_state_t>> event_queue_t;
  std::set<std::pair<uint16_t, uint16_t>> sent_tasks;  // Pair of (robot_id, task_id)

// Private functions
private:
  void printEventTable() const {
    printf("-------------------------------------------------\n");
    printf("| Event ID |   Type   | Finished | Processed |\n");
    printf("-------------------------------------------------\n");
    for (const auto& event : events_) {
        if (!event) continue; // Skip uninitialized events
        printf("|   %3d   |     %s     |    %d     |     %d     |\n",
               event->id_,
               event->type_ == A ? "A" : "B",
               event->finished_,
               event->processed_);
    }
    printf("-------------------------------------------------\n");
  }

  void printEventQueue(const event_queue_t& event_queue) const {
    printf("------------------------------------------------------------\n");
    printf("| Event ID |   Type   |       State       |   Position (X, Y)   |\n");
    printf("------------------------------------------------------------\n");
    for (const auto& e_es_pair : event_queue) {
        const Event* event = e_es_pair.first;
        message_event_state_t state = e_es_pair.second;

        if (!event) continue; // Skip null events

        // Convert state to string
        const char* state_str;
        switch (state) {
            case MSG_EVENT_INVALID: state_str = "INVALID"; break;
            case MSG_EVENT_GPS_ONLY: state_str = "GPS_ONLY"; break;
            case MSG_EVENT_NEW: state_str = "NEW"; break;
            case MSG_EVENT_WON: state_str = "WON"; break;
            case MSG_EVENT_DONE: state_str = "DONE"; break;
            case MSG_QUIT: state_str = "QUIT"; break;
            case MSG_EVENT_BEING_HANDLED: state_str = "BEING_HANDLED"; break;
            case MSG_EVENT_ACTIVE: state_str = "ACTIVE"; break;
            default: state_str = "UNKNOWN"; break;
        }

        printf("|   %3d   |     %s     |    %13s    |   (%.2f, %.2f)   |\n",
               event->id_,
               event->type_ == A ? "A" : "B",
               state_str,
               event->pos_.x,
               event->pos_.y);
    }
    printf("------------------------------------------------------------\n");
  }

  
  void addEvent() {
    events_.push_back(unique_ptr<Event>(new Event{next_event_id_++})); // add to list
    assert(num_active_events_ < NUM_ACTIVE_EVENTS); // check max. active events not reached
    printf("Added Event %d (Type: %s) at Position (%.2f, %.2f).\n",
            next_event_id_-1, 
            events_.back()->type_ == A ? "A" : "B",
            events_.back()->pos_.x, 
            events_.back()->pos_.y);
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
      msg->event_id = event->id_;
      msg->event_x = event->pos_.x;
      msg->event_y = event->pos_.y;
      msg->event_type = event->type_;
      msg->event_finished = event->finished_;
    }
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
    for (auto &event : events_) {
      if (event->finished_ && !event->processed_) {
        num_events_handled_++;
        num_active_events_--;
        printf("Marked Event %d as DONE.\n", event->id_);
        event->processed_ = 1;
        event_queue.emplace_back(event.get(), MSG_EVENT_DONE);
        if (num_active_events_ < NUM_ACTIVE_EVENTS) {
          addEvent();
        }
      }
    }  
    for (auto &event : events_) {
      //if (!event->is_done() && processed_events.find(event->id_) == processed_events.end()) {
      if (!event->is_done()) {  
        // Add unfinished events to the queue as NEW
        event_queue.emplace_back(event.get(), MSG_EVENT_NEW);
        //printf("Added Event %d to the event_queue. (Type: %s, Pos: [%.2f, %.2f])\n",
        //        event->id_, event->type_ == A ? "A" : "B", event->pos_.x, event->pos_.y);
      } 
    }
  }

  // Calculate total distance travelled by robots
  void statTotalDistance() {
    for (int i=0; i<NUM_ROBOTS; ++i) {
      const double *robot_pos = getRobotPos(i);
      double delta[2] = {
        robot_pos[0] - stat_robot_prev_pos_[i][0],
        robot_pos[1] - stat_robot_prev_pos_[i][1]
      };
      stat_total_distance_ += sqrt(delta[0]*delta[0] + delta[1]*delta[1]);
      stat_robot_prev_pos_[i][0] = robot_pos[0];
      stat_robot_prev_pos_[i][1] = robot_pos[1];
    }
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
    t_next_gps_tick_ = 0;

    num_events_handled_ = 0;
    stat_total_distance_ = 0.0;

    // add the first few events
    for (int i=0; i<NUM_ACTIVE_EVENTS; ++i) {
      addEvent();
    }
    printf("There are %d active events.\n", num_active_events_);

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

    super_receiver_ = wb_robot_get_device("sup_receiver");
    wb_receiver_enable(super_receiver_, RX_PERIOD);
    wb_receiver_set_channel(super_receiver_, 12);
  }

  //Do a step
  bool step(uint64_t step_size) {
    
    clock_ += step_size;

    // Events that will be announced next or that have just been assigned/done
    event_queue_t event_queue;
    //std::set<int> processed_events;

    markEventsDone(event_queue);

    // // ** Add a random new event, if the time has come
    // assert(t_next_event_ > 0);
    // if (clock_ >= t_next_event_ && num_active_events_ < NUM_ACTIVE_EVENTS) {
    //   addEvent();
    // }
 
    // Send and receive messages
    robot_message_t r_msg; // inbound
    for (int i=0;i<NUM_ROBOTS;i++) {  
      // Check if we're receiving data
      if (wb_receiver_get_queue_length(receivers_[i]) > 0) {
        assert(wb_receiver_get_queue_length(receivers_[i]) > 0);
        assert(wb_receiver_get_data_size(receivers_[i]) == sizeof(robot_message_t));
        const robot_message_t *rmsg = (robot_message_t*) wb_receiver_get_data(receivers_[i]);
        
        // Make a copy of the message
        memcpy(&r_msg, rmsg, sizeof(robot_message_t));
        wb_receiver_next_packet(receivers_[i]);

        assert(r_msg.robot_id == i);

        Event* event = events_.at(r_msg.event_id).get();

        event->finished_ = r_msg.finished;

        if (r_msg.finished) {
          //event->finished_ = r_msg->finished;
          printf("Event %d was finished.\n", event->id_);
          //markEventsDone(event_queue, processed_events);
        }
      }
    }

    // outbound
    message_t msg;
    bool is_gps_tick = false;

    for (int i=0;i<NUM_ROBOTS;i++) {
      // Send updates to the robot
      while (wb_emitter_get_channel(emitter_) != i+1)
      wb_emitter_set_channel(emitter_, i+1);
      
      // if (event_queue.empty()) {
      //    printf("Event Queue is EMPTY. No events to send to robots.\n");
      // } else {
      //    printf("Event Queue contains %d events.\n", event_queue.size());
      // } 

      for (const auto& e_es_tuple : event_queue) {
        const Event* event = e_es_tuple.first;
        const message_event_state_t event_state = e_es_tuple.second;
        if (event_state == MSG_EVENT_NEW) {
          // Check if this task was already sent to this robot
          if (sent_tasks.find({i, event->id_}) != sent_tasks.end()) {
              continue;  // Skip if already sent
          }
          
          buildMessage(i, event, event_state, &msg);
          while (wb_emitter_get_channel(emitter_) != i+1) wb_emitter_set_channel(emitter_, i+1);
          printf("Sending message to channel %d\n", wb_emitter_get_channel(emitter_));
          //printf("Sending robot position to Robot %d : [x=%d, y=%d, heading=%d]\n", msg.robot_id, msg.robot_x, msg.robot_y, msg.heading);
          printf("Sending Event message to Robot %d: [Event ID=%d, Event X=%.2f, Event Y=%.2f, Event Type=%s]\n",
                msg.robot_id, msg.event_id, msg.event_x, msg.event_y,
                msg.event_type == A ? "A" : "B");
          wb_emitter_send(emitter_, &msg, sizeof(message_t));
          // Mark this task as sent
          sent_tasks.insert({i, event->id_});
        }
      } 
    }

    // Print the events table 
    printEventTable();

    // Print the event queue
    //printEventQueue(event_queue);

    // Keep track of distance travelled by all robots
    statTotalDistance();

    // Time to end the experiment?
    if (num_events_handled_ >= TOTAL_EVENTS_TO_HANDLE ||(MAX_RUNTIME > 0 && clock_ >= MAX_RUNTIME)) {
      for(int i=0;i<NUM_ROBOTS;i++){
          buildMessage(i, NULL, MSG_QUIT, &msg);
          wb_emitter_set_channel(emitter_, i+1);
          wb_emitter_send(emitter_, &msg, sizeof(message_t));
      }
      double clock_s = ((double) clock_) / 1000.0;
      double ehr = ((double) num_events_handled_) / clock_s;
      double perf = ((double) num_events_handled_) / stat_total_distance_;
      
      printf("Handled %d events in %d seconds, events handled per second = %.2f\n",
             num_events_handled_, (int) clock_ / 1000, ehr);
      printf("Performance: %f\n", perf);
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
    if (g_event_nodes[i] == NULL) {
      printf("Node %s not found! Check the Webots world file.\n", node_name);
      exit(1); // Fail fast if a node is missing
    }
  }
  printf("Every event node was linked successfully.\n");
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
  while (wb_robot_step(STEP_SIZE) != -1)
  {
    printf("SUPERVISOR LOOP\n");
    if (!supervisor.step(STEP_SIZE)) break; //break at return = false
  }
  //wb_supervisor_simulation_reset_physics();
  wb_robot_cleanup();
  exit(0);
  return 0;
}