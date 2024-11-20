#include <assert.h>
#include <bitset>
#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <vector>
#include <memory>
#include <time.h>

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

#define STEP_SIZE 64 // Simulation step size
#define AUCTION_TIMEOUT 1000 // Number of steps for auction timeout
#define EVENT_RANGE 0.1 // Distance threshold for task completion
#define EVENT_TIMEOUT 10000 // Auction timeout duration
#define EVENT_GENERATION_DELAY 1000 // Average delay between new events

#define GPS_INTERVAL 500 // GPS update interval
#define NUM_ACTIVE_EVENTS 5 // Number of active events in the world
#define TOTAL_EVENTS_TO_HANDLE 50 // Number of tasks to complete
#define MAX_RUNTIME (3 * 60 * 1000) // Maximum runtime in milliseconds

// Robot specializations: 0 for Task A, 1 for Task B
vector<int> robot_specializations = {0, 0, 1, 1, 1};

// Event class
class Event {
public:
    uint16_t id_; // Event ID
    Point2d pos_; // Event position
    int task_type_; // 0 for Task A, 1 for Task B
    WbNodeRef node_; // Event node reference
    uint16_t assigned_to_; // Assigned robot ID
    uint64_t t_announced_; // Announcement time
    uint64_t t_done_; // Completion time
    bitset<MAX_ROBOTS> bids_in_; // Bids received
    uint16_t best_bidder_; // ID of robot with the best bid
    double best_bid_; // Best bid value
    int bidder_index; // Task list index for the assigned robot

    Event(uint16_t id) : id_(id), pos_(rand_coord(), rand_coord()), task_type_((RAND < 1.0 / 3.0) ? 0 : 1),
                         assigned_to_(-1), t_announced_(-1), t_done_(-1), best_bidder_(-1), best_bid_(0.0) {
        node_ = g_event_nodes_free.back();
        g_event_nodes_free.pop_back();
        double event_node_pos[3] = {pos_.x, pos_.y, 0.01};
        wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(node_, "translation"), event_node_pos);
    }

    bool is_assigned() const { return assigned_to_ != (uint16_t)-1; }
    bool was_announced() const { return t_announced_ != (uint64_t)-1; }
    bool has_bids() const { return best_bidder_ != (uint16_t)-1; }
    bool is_done() const { return t_done_ != (uint64_t)-1; }

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
        double event_node_pos[3] = {-5, -5, 0.1};
        wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(node_, "translation"), event_node_pos);
        g_event_nodes_free.push_back(node_);
    }
};

// Supervisor class
class Supervisor {
private:
    uint64_t clock_;
    uint16_t next_event_id_;
    vector<unique_ptr<Event>> events_;
    uint16_t num_active_events_;
    uint64_t t_next_event_;
    WbNodeRef robots_[MAX_ROBOTS];
    WbDeviceTag emitter_;
    WbDeviceTag receivers_[MAX_ROBOTS];
    vector<double> robot_energies_; // Remaining energy for each robot

    void addEvent() {
        events_.push_back(make_unique<Event>(next_event_id_++));
        num_active_events_++;
        t_next_event_ = clock_ + EVENT_GENERATION_DELAY;
    }

    void linkRobot(uint16_t id) {
        const char kRobotNameFormat[] = "e-puck%d";
        const char kReceiverNameFormat[] = "rec%d";
        char node_name[16];

        sprintf(node_name, kRobotNameFormat, id);
        robots_[id] = wb_supervisor_node_get_from_def(node_name);
        sprintf(node_name, kReceiverNameFormat, id);
        receivers_[id] = wb_robot_get_device(node_name);
        wb_receiver_enable(receivers_[id], STEP_SIZE);
    }

    double calculateBid(uint16_t robot_id, const Event* event) {
        const double* robot_pos = wb_supervisor_field_get_sf_vec3f(wb_supervisor_node_get_field(robots_[robot_id], "translation"));
        double distance = sqrt(pow(robot_pos[0] - event->pos_.x, 2) + pow(robot_pos[1] - event->pos_.y, 2));

        if (robot_specializations[robot_id] != event->task_type_) {
            distance *= 1.5; // Penalize non-specialized robots
        }

        return distance;
    }

    void assignTasks() {
        for (auto& event : events_) {
            if (event->is_assigned() || !event->was_announced()) continue;

            double best_bid = INFINITY;
            uint16_t best_robot = -1;

            for (uint16_t robot_id = 0; robot_id < MAX_ROBOTS; robot_id++) {
                if (robot_energies_[robot_id] <= 0) continue; // Skip robots without energy

                double bid = calculateBid(robot_id, event.get());
                if (bid < best_bid) {
                    best_bid = bid;
                    best_robot = robot_id;
                }
            }

            if (best_robot != (uint16_t)-1) {
                event->updateAuction(best_robot, best_bid, 0);
                event->assigned_to_ = best_robot;
                printf("Event %d assigned to robot %d\n", event->id_, best_robot);
            }
        }
    }

public:
    Supervisor() : events_(), num_active_events_(0), next_event_id_(0), t_next_event_(0), clock_(0), robot_energies_(MAX_ROBOTS, 120.0) {}

    void reset() {
        clock_ = 0;
        events_.clear();
        num_active_events_ = 0;
        for (uint16_t i = 0; i < MAX_ROBOTS; i++) {
            linkRobot(i);
        }
        emitter_ = wb_robot_get_device("sup_emitter");
        for (int i = 0; i < NUM_ACTIVE_EVENTS; ++i) addEvent();
    }

    bool step() {
        clock_ += STEP_SIZE;

        if (clock_ >= t_next_event_ && num_active_events_ < NUM_ACTIVE_EVENTS) addEvent();
        assignTasks();

        if (clock_ >= MAX_RUNTIME || num_active_events_ >= TOTAL_EVENTS_TO_HANDLE) return false;
        return true;
    }
};

// Main function
int main() {
    wb_robot_init();
    Supervisor supervisor;
    supervisor.reset();

    while (supervisor.step()) {
        wb_robot_step(STEP_SIZE);
    }

    wb_robot_cleanup();
    return 0;
}
