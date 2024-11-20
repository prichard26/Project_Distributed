#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <string.h>
#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include "../auct_super/message.h"

// Robot specifications and constants
#define TIME_STEP 64
#define MAX_SPEED 6.28
#define WHEEL_RADIUS 0.0205
#define AXLE_LENGTH 0.052
#define NUM_ROBOTS 5
#define MAX_TASKS 99
#define INVALID -999

typedef enum {
    STAY = 1,
    GO_TO_GOAL = 2,
    OBSTACLE_AVOID = 3,
} robot_state_t;

// Global variables
WbDeviceTag left_motor, right_motor, emitter_tag, receiver_tag;
WbDeviceTag ds[8];
double my_pos[3];
double target[MAX_TASKS][3];
int robot_id, target_valid;
robot_state_t state;
double energy = 120.0; // Robot's energy level

// Helper functions
double rnd(void) {
    return ((double)rand()) / ((double)RAND_MAX);
}

double dist(double x0, double y0, double x1, double y1) {
    return sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
}

// State management
void update_state(int sum_distances) {
    if (sum_distances > 500 && state == GO_TO_GOAL) {
        state = OBSTACLE_AVOID;
    } else if (target_valid) {
        state = GO_TO_GOAL;
    } else {
        state = STAY;
    }
}

// Task bidding and updates
void receive_updates() {
    while (wb_receiver_get_queue_length(receiver_tag) > 0) {
        const message_t *msg = wb_receiver_get_data(receiver_tag);
        wb_receiver_next_packet(receiver_tag);

        if (msg->robot_id != robot_id) continue;

        // Handle task announcements
        if (msg->event_state == MSG_EVENT_NEW) {
            double best_cost = INFINITY;
            int target_list_length = 0;

            while (target[target_list_length][2] != INVALID) target_list_length++;

            int indx = 0;
            for (int i = 0; i <= target_list_length; ++i) {
                double cost = dist(my_pos[0], my_pos[1], msg->event_x, msg->event_y);
                if (i > 0) cost += dist(target[i - 1][0], target[i - 1][1], msg->event_x, msg->event_y);
                if (i < target_list_length) cost += dist(msg->event_x, msg->event_y, target[i][0], target[i][1]);
                if (i > 0 && i < target_list_length) cost -= dist(target[i - 1][0], target[i - 1][1], target[i][0], target[i][1]);
                if (cost < best_cost) {
                    best_cost = cost;
                    indx = i;
                }
            }

            bid_t my_bid = {robot_id, msg->event_id, best_cost, indx};
            wb_emitter_set_channel(emitter_tag, robot_id + 1);
            wb_emitter_send(emitter_tag, &my_bid, sizeof(bid_t));
        }
    }
}

// Navigation
void compute_go_to_goal(int *msl, int *msr) {
    float a = target[0][0] - my_pos[0];
    float b = target[0][1] - my_pos[1];
    float x = a * cosf(my_pos[2]) - b * sinf(my_pos[2]);
    float y = a * sinf(my_pos[2]) + b * cosf(my_pos[2]);

    float Ku = 0.2, Kw = 10.0;
    float range = sqrtf(x * x + y * y);
    float bearing = atan2(y, x);
    float u = Ku * range * cosf(bearing);
    float w = Kw * range * sinf(bearing);

    *msl = 50 * (u - AXLE_LENGTH * w / 2.0) / WHEEL_RADIUS;
    *msr = 50 * (u + AXLE_LENGTH * w / 2.0) / WHEEL_RADIUS;
}

void compute_avoid_obstacle(int *msl, int *msr, int distances[]) {
    int d1 = 0, d2 = 0;
    for (int i = 0; i < 8; i++) {
        d1 += distances[i] * ((i < 4) ? 1 : -1);
        d2 += distances[i] * ((i >= 4) ? 1 : -1);
    }
    *msl = d1 + 400;
    *msr = d2 + 400;
}

// Motion control
void run() {
    int msl = 0, msr = 0;
    int distances[8];
    int sum_distances = 0;

    for (int i = 0; i < 8; i++) {
        distances[i] = wb_distance_sensor_get_value(ds[i]);
        sum_distances += distances[i];
    }

    receive_updates();
    update_state(sum_distances);

    if (energy <= 0) state = STAY;

    switch (state) {
        case STAY:
            msl = 0;
            msr = 0;
            break;
        case GO_TO_GOAL:
            compute_go_to_goal(&msl, &msr);
            break;
        case OBSTACLE_AVOID:
            compute_avoid_obstacle(&msl, &msr, distances);
            break;
    }

    wb_motor_set_velocity(left_motor, msl * MAX_SPEED / 1000.0);
    wb_motor_set_velocity(right_motor, msr * MAX_SPEED / 1000.0);
    energy -= sqrt(msl * msl + msr * msr) * TIME_STEP * 0.001;
}

// Initialization
void reset() {
    wb_robot_init();
    left_motor = wb_robot_get_device("left wheel motor");
    right_motor = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);

    char s[] = "ps0";
    for (int i = 0; i < 8; i++) {
        ds[i] = wb_robot_get_device(s);
        wb_distance_sensor_enable(ds[i], TIME_STEP);
        s[2]++;
    }

    char *robot_name = (char *)wb_robot_get_name();
    sscanf(robot_name, "e-puck%d", &robot_id);

    emitter_tag = wb_robot_get_device("emitter");
    receiver_tag = wb_robot_get_device("receiver");
    wb_receiver_enable(receiver_tag, TIME_STEP);
    wb_emitter_set_channel(emitter_tag, robot_id + 1);

    for (int i = 0; i < MAX_TASKS; i++) {
        target[i][0] = 0;
        target[i][1] = 0;
        target[i][2] = INVALID;
    }
    target_valid = 0;
    state = STAY;
}

int main() {
    reset();
    while (wb_robot_step(TIME_STEP) != -1) {
        run();
    }
    wb_robot_cleanup();
    return 0;
}
