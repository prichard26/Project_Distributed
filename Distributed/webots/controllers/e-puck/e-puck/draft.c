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

// Print incoming messages (receivers)
    bid_t* pbid; // inbound
    for (int i = 0; i < NUM_ROBOTS; i++) {
        int queue_length = wb_receiver_get_queue_length(receivers_[i]);
        printf("Receiver queue length for robot %d: %d\n", i, queue_length);

        if (queue_length > 0) {
            printf("We are receiving data from robot %d!\n", i);
            assert(wb_receiver_get_data_size(receivers_[i]) == sizeof(bid_t));

            pbid = (bid_t*)wb_receiver_get_data(receivers_[i]);
            printf("Received bid: robot_id=%d, event_id=%d, value=%.2f, event_index=%d\n", pbid->robot_id, pbid->event_id, pbid->value, pbid->event_index);

            Event* event = events_.at(pbid->event_id).get();
            event->updateAuction(pbid->robot_id, pbid->value, pbid->event_index);

            if (event->is_assigned()) {
                printf("Event %d assigned to robot %d\n", event->id_, event->assigned_to_);
                event_queue.emplace_back(event, MSG_EVENT_WON);
                auction = NULL;
            }

            wb_receiver_next_packet(receivers_[i]);
        }
    }

// Outgoing messages (emitter)
    message_t msg;
    bool is_gps_tick = false;

    if (clock_ >= t_next_gps_tick_) {
        is_gps_tick = true;
        t_next_gps_tick_ = clock_ + GPS_INTERVAL;
        printf("GPS tick triggered. Next GPS tick: %" PRIu64 "\n", t_next_gps_tick_);
    }

    for (int i = 0; i < NUM_ROBOTS; i++) {
        // Set emitter channel
        while (wb_emitter_get_channel(emitter_) != i + 1)
            wb_emitter_set_channel(emitter_, i + 1);

        // Send GPS updates
        if (is_gps_tick) {
            buildMessage(i, NULL, MSG_EVENT_GPS_ONLY, &msg);
            wb_emitter_send(emitter_, &msg, sizeof(message_t));
            printf("Sent GPS update to robot %d\n", i);
        }

        // Send event updates
        for (const auto& e_es_tuple : event_queue) {
            const Event* event = e_es_tuple.first;
            const message_event_state_t event_state = e_es_tuple.second;

            if (event->is_assigned() && event->assigned_to_ != i) continue;

            buildMessage(i, event, event_state, &msg);
            wb_emitter_send(emitter_, &msg, sizeof(message_t));
            printf("Sent event update to robot %d: event_id=%d, event_state=%d\n", i, event->id_, event_state);
        }
    }


void handleAuctionEvents(event_queue_t& event_queue) {
    // For each unassigned event
    for (auto& event : events_) {
      if (event->is_assigned()) continue;

      // Send announce, if new
      // IMPL DETAIL: Only allow one auction at a time.
      if (!event->was_announced() && !auction) {
        event->t_announced_ = clock_;
        event_queue.emplace_back(event.get(), MSG_EVENT_NEW); 
        auction = event.get();
        printf("A event %d announced\n", event->id_);

      // End early or restart, if timed out
      } else if (clock_ - event->t_announced_ > EVENT_TIMEOUT) {
        // End early if we have any bids at all
        if (event->has_bids()) {
          // IMPLEMENTATION DETAIL: If about to time out, assign to
          // the highest bidder or restart the auction if there is none.
          event->assigned_to_ = event->best_bidder_;
          event_queue.emplace_back(event.get(), MSG_EVENT_WON); // FIXME?
          auction = NULL;
          printf("W robot %d won event %d\n", event->assigned_to_, event->id_);

        // Restart (incl. announce) if no bids
        } else {
          // (reannounced in next iteration)
          event->restartAuction();
          if (auction == event.get())
            auction = NULL;
        }
      }
    }
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
    double pos[3] = {x, y, 0.005};
    return wb_supervisor_field_set_sf_vec3f(f_pos, pos);
  }

  // Marks one event as done, if one of the robots is within the range
  void markEventsDone(event_queue_t& event_queue) {
    for (auto& event : events_) {
      if (!event->is_assigned() || event->is_done())
        continue;
      
      const double *robot_pos = getRobotPos(event->assigned_to_);
      Point2d robot_pos_pt(robot_pos[0], robot_pos[1]);
      double dist = event->pos_.Distance(robot_pos_pt);

      if (dist <= EVENT_RANGE) {
        printf("D robot %d reached event %d\n", event->assigned_to_,
          event->id_);
        num_events_handled_++;
        event->markDone(clock_);
        num_active_events_--;
        event_queue.emplace_back(event.get(), MSG_EVENT_DONE);
      }
    }
  }