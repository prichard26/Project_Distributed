  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * file:        Supervidor
 * author:      
 * description: Supervisor for Disributed market-based task allocation 
 *
 * $Date$     20.11.2024
 * $Author$   Paul Richard
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// =============================================== INCLUDES ===============================================

#include <assert.h>
#include <bitset>
#include <cstdlib>
#include <cstdio>
#include <cmath>

#include <vector>
#include <memory>
#include <time.h>       /* time_t, struct tm, difftime, time, mktime */
#include <inttypes.h>

using namespace std;

#include "Point2d.h"
#include "message.h"

#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/supervisor.h>
#include <webots/robot.h>

// =========================================== GLOBAL VARIABLES ===========================================

#define DBG(x) printf x
#define RAND ((float) rand()/RAND_MAX)

#define MAX_ROBOTS 5
#define MAX_EVENTS 10

#define STEP_SIZE 64            // simulation step size
#define AUCTION_TIMEOUT 1000    // number of steps after which an auction stops

#define EVENT_RANGE (0.1)      // distance within which a robot must come to do event
#define EVENT_TIMEOUT (10000)   // ticks until an event auction runs out
#define EVENT_GENERATION_DELAY (1000) // average time between events ms (expo distribution)

#define GPS_INTERVAL (500)

#define NUM_ROBOTS 5                 // Change this also in the epuck_crown.c!
#define NUM_ACTIVE_EVENTS 10          // number of active events
#define TOTAL_EVENTS_TO_HANDLE 100   // Events after which simulation stops or...
#define MAX_RUNTIME (3*60*1000)      // ... total runtime after which simulation stops

// Robot specializations: 0 for Task A, 1 for Task B
vector<int> robot_specializations = {0, 0, 1, 1, 1}; // 1/3 for Task A, 2/3 for Task B
WbNodeRef g_event_nodes[MAX_EVENTS];
vector<WbNodeRef> g_event_nodes_free;

// ============================================ TOOL FUNCTIONS ============================================

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
  double min = -0.45;
  double max = 0.45;
  return min + (max - min) * RAND;
}

double expovariate(double mu) {
  double uniform = RAND;
  while (uniform < 1e-7) uniform = RAND;
  return -log(uniform) * mu;
}

// ============================================= EVENT CLASS ==============================================

class Event {
public:
  uint16_t id_;          // Event id
  Point2d pos_;          // Event pos
  int task_type_;        // 0 for task A and 1 for task B
  WbNodeRef node_;       // Event node ref
  uint16_t assigned_to_; // ID of the robot that will handle this event

  // Auction data
  uint64_t t_announced_;        // Announcement time
  uint64_t t_done_;             // Time at which the assigned robot reached the event
  bitset<NUM_ROBOTS> bids_in_;  // Bid received
  uint16_t best_bidder_;        // ID of the robot that had the best bid so far
  double best_bid_;             // Value of the best bid (lowest value)
  int bidder_index;             // Task list index for the assigned robot

  //Event creation
  Event(uint16_t id) : id_(id), pos_(rand_coord(), rand_coord()), task_type_((RAND<1.0/3.0)? 0 : 1),
                       assigned_to_(-1), t_announced_(-1), t_done_(-1), best_bidder_(-1), best_bid_(0.0){
    node_ = g_event_nodes_free.back();  // Place node
    g_event_nodes_free.pop_back();
    
    double event_node_pos[3] = {pos_.x, pos_.y, 0.01}; //Place event in arena
    wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(node_,"translation"),event_node_pos);
  }

  bool is_assigned() const { return assigned_to_ != (uint16_t) -1; }
  bool was_announced() const { return t_announced_ != (uint64_t) -1; }
  bool has_bids() const { return best_bidder_ != (uint16_t) -1; }
  bool is_done() const { return t_done_ != (uint64_t) -1; }

  // Check if event can be assigned
  void updateAuction(uint16_t bidder, double bid, int index) {
    printf("Robot %d bidding on event %d with bid %.2f\n", bidder, id_, bid);
    if (bid >= 0.0 && (!has_bids() || bid < best_bid_)) {
      best_bidder_ = bidder;
      best_bid_ = bid;
      bidder_index = index;  
      printf("Robot %d now has the best bid %.2f for event %d\n", bidder, bid, id_);
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
    wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(node_,"translation"),event_node_pos);
    g_event_nodes_free.push_back(node_);
  }
};

// =========================================== SUPERVISOR CLASS ===========================================

class Supervisor {
private:
  uint64_t clock_;
  uint16_t next_event_id_;
  vector<unique_ptr<Event> > events_;
  uint16_t num_active_events_;
  uint64_t t_next_event_;

  Event* auction;                     // the event currently being auctioned
  uint64_t t_next_gps_tick_;
  uint16_t num_events_handled_;       // total number of events handled
  double stat_total_distance_;        // total distance traveled
  double stat_robot_prev_pos_[NUM_ROBOTS][2];

  WbNodeRef robots_[NUM_ROBOTS];
  WbDeviceTag emitter_;
  WbDeviceTag receivers_[NUM_ROBOTS];
  vector<double> robot_energies_;     // Remaining energy for each robot

  typedef vector<pair<Event*, message_event_state_t>> event_queue_t;

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
    printf("Linking robot: %s\n", node_name);  // Debugging line

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

// Helper function to check if a position is at least MIN_DISTANCE away from all existing positions
bool isValidPosition(double x, double y, int robot_id) {
    Point2d candidate(x, y);
    for (int i = 0; i < robot_id; ++i) {
        const double *existing_pos = getRobotPos(i);
        Point2d existing(existing_pos[0], existing_pos[1]);
        if (candidate.Distance(existing) < 0.2) { // 20 cm
            return false;
        }
    }
    return true;
}

// Helper function to place a robot ensuring minimum distance
void placeRobotWithMinDistance(int robot_id) {
    double x, y;
    do {
        x = rand_coord();
        y = rand_coord();
    } while (!isValidPosition(x, y, robot_id));

    setRobotPos(robot_id, x, y);
    stat_robot_prev_pos_[robot_id][0] = x;
    stat_robot_prev_pos_[robot_id][1] = y;
}

public:
  Supervisor() : events_(MAX_EVENTS){}
  
  // Reset robots & events
  void reset() {
    clock_ = 0;

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
      placeRobotWithMinDistance(i); // Ensure placement respects minimum distance
    }

    // initialize the emitter
    emitter_ = wb_robot_get_device("sup_emitter");
    if (!emitter_) {
      DBG(("Missing supervisor emitter!\n"));
      exit(1);
    }
  }

  //Do a step
//Do a step
bool step(uint64_t step_size) {
    printf("==== Step Function Called ====\n");

    // Update clock
    clock_ += step_size;
    printf("Clock updated: %" PRIu64 "\n", clock_);

    // Print number of active events
    printf("Number of active events: %d\n", num_active_events_);

    // Print upcoming event time
    printf("Next event will be added at clock: %" PRIu64 "\n", t_next_event_);

    // Print auction status
    if (auction) {
        printf("Current auction for event ID: %d\n", auction->id_);
    } else {
        printf("No active auction at the moment.\n");
    }

    // Print robot positions
    for (int i = 0; i < NUM_ROBOTS; i++) {
        const double* pos = getRobotPos(i);
        printf("Robot %d position: x=%.2f, y=%.2f\n", i, pos[0], pos[1]);
    }

    // Events that will be announced next or that have just been assigned/done
    event_queue_t event_queue;

    // Mark events as done
    markEventsDone(event_queue);

    // Add new event if the time has come
    assert(t_next_event_ > 0);
    if (clock_ >= t_next_event_ && num_active_events_ < NUM_ACTIVE_EVENTS) {
      printf("Adding a new event. Current clock: %" PRIu64 ", Next event time: %" PRIu64 "\n", clock_, t_next_event_);
        addEvent();
    }
    '''
    // Handle auction events
    handleAuctionEvents(event_queue);
    '''
    // Keep track of distance travelled by all robots
    statTotalDistance();

    // Check if the experiment should end
    if (num_events_handled_ >= TOTAL_EVENTS_TO_HANDLE || (MAX_RUNTIME > 0 && clock_ >= MAX_RUNTIME)) {
        for (int i = 0; i < NUM_ROBOTS; i++) {
            buildMessage(i, NULL, MSG_QUIT, &msg);
            wb_emitter_set_channel(emitter_, i + 1);
            wb_emitter_send(emitter_, &msg, sizeof(message_t));
        }
        double clock_s = ((double)clock_) / 1000.0;
        double ehr = ((double)num_events_handled_) / clock_s;
        double perf = ((double)num_events_handled_) / stat_total_distance_;

        printf("Handled %d events in %d seconds, events handled per second = %.2f\n",
               num_events_handled_, (int)clock_ / 1000, ehr);
        printf("Performance: %f\n", perf);
        return false;
    } else {
        return true;
    }
  }
// << step() <<
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
  wb_supervisor_simulation_reset_physics();
  wb_robot_cleanup();
  exit(0);
  return 0;

}