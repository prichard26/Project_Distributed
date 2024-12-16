  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * file:        epuck_crown.c
 * author:      
 * description: E-puck file for market-based task allocations (DIS lab05)
 *
 * $Revision$	february 2016 by Florian Maushart
 * $Date$
 * $Author$      Last update 2024 by Wanting Jin
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>

#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/distance_sensor.h>
#include <webots/radio.h>
#include <webots/motor.h>
#include <webots/gps.h>
  
#include <webots/supervisor.h> 

#include "../auct_super/message.h" 
#define MAX_SPEED_WEB      6.28    // Maximum speed webots
WbDeviceTag left_motor; //handler for left wheel of the robot
WbDeviceTag right_motor; //handler for the right wheel of the robot


#define DEBUG 1
#define TIME_STEP           64      // Timestep (ms)
#define RX_PERIOD           2    // time difference between two received elements (ms) (1000)

#define AXLE_LENGTH         0.052   // Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS     0.00628 // Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS        0.0205  // Wheel radius (meters)
#define DELTA_T             TIME_STEP/1000   // Timestep (seconds)
#define MAX_SPEED           800     // Maximum speed
#define MAX_VELOCITY        0.5 
#define MAX_MOTOR_SPEED     24.39   // Max motor speed in rad/s

#define INVALID          -999
#define BREAK            -999 //for physics plugin
#define INF               1000

#define NUM_ROBOTS 5 // Change this also in the supervisor!
#define NUM_TASKS 10
#define SR_COM_RANGE 0.3
#define TASK_COMPLETION_RANGE 0.1


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Collective decision parameters */

#define STATECHANGE_DIST 1200   // minimum value of all sensor inputs combined to change to obstacle avoidance mode

typedef enum {
    STAY            = 1,
    GO_TO_GOAL      = 2,                    // Initial state aliases
    OBSTACLE_AVOID  = 3,
    RANDOM_WALK     = 4,
    HANDLING_TASK   = 5,
    FINISHED_TASK   = 6,
} robot_state_t;

#define DEFAULT_STATE (STAY)

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* e-Puck parameters */

#define NB_SENSORS           8
#define BIAS_SPEED           400

// Weights for the Braitenberg algorithm
// NOTE: Weights from reynolds2.h
int Interconn[16] = {17,29,34,10,8,-38,-56,-76,-72,-58,-36,8,10,36,28,18};

// Proximity and radio handles
WbDeviceTag emitter_tag, receiver_tag;
static WbDeviceTag ds[NB_SENSORS];  // Handle for the infrared distance sensors
WbDeviceTag sr_emitter_tag, sr_receiver_tag; //  Handle for inter robot communication
WbDeviceTag gps; // Handle for gps

// Structure that contains information about a task to be broadcasted to the robots from the supervisor
typedef struct {
    uint16_t task_id;
    Event_type task_type; // A or B
    int finished;
} task_message_t;

// Structure that contains information about a task 
typedef struct {
    uint16_t task_id;
    double task_x, task_y;
    Event_type task_type; 
    double best_bid_value; // Lowest bid value known
    uint16_t best_bid_robot; // Robot ID of best bidder
    double bid_values[NUM_ROBOTS]; // Table to hold the values of the bids of each robot
    int is_completed;
} active_task_t;

typedef struct {
    uint16_t task_id;
    double task_x, task_y;
    Event_type task_type;
    int is_completed;
    int in_active;
    //uint16_t assigned_to;   // Id of the robot to which the task is assigned
} task_t;

// The state variables
int clock;
uint16_t robot_id;          // Unique robot ID
Event_type robot_type;
robot_state_t state;        // State of the robot
double my_pos[2];           // X, Z, Theta of this robot
//double prev_pos[2];         // previous X, Z, and Theta of this robot
double my_heading;          // Theta of this robot
//double prev_heading;        // previous Theta of this robot
//const double *gps_pos;          // Variable to hold the values from the gps (X, Y, and Z)
bool target_valid;          // boolean; whether we are supposed to go to the target
task_t all_tasks[99];       // x and z coordinates of target position (max 99 targets)
active_task_t active_tasks[NUM_TASKS];    // Array containing all the tasks present on the field
task_t goal_tasks[3];
int lmsg, rmsg;             // Communication variables
int indx;                   // Event index to be sent to the supervisor

task_t current_goal_task = { .task_id = INVALID, .task_x = 0, .task_y = 0, .task_type = INVALID, .is_completed = 0 };

float buff[99];             // Buffer for physics plugin

double stat_max_velocity;

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* helper functions */

// Generate random number in [0,1]
double rnd(void) {
  return ((double)rand())/((double)RAND_MAX);
}

void limit(int *number, int limit) {
    if (*number > limit)
        *number = limit;
    if (*number < -limit)
        *number = -limit;
}

double dist(double x0, double y0, double x1, double y1) {
    return sqrt((x0-x1)*(x0-x1) + (y0-y1)*(y0-y1));
}

void sanity_check(active_task_t *task, const char *label) {
    if (task->task_id == INVALID) {
        printf("Error: Task in %s is INVALID!\n", label);
        exit(0);
    }
}

double compute_heading(const double *pos, double *prev_pos, double prev_heading) {
    double x = pos[0] - prev_pos[0];
    double y = pos[1] - prev_pos[1];

    // If the robot hasn't moved, retain the previous heading
    if (fabs(x) < 1e-3 && fabs(y) < 1e-3) {
        return prev_heading;
    }

    if ((x == 0) && (y == 0)) return prev_heading;

    double heading =  atan2(y, x);
    if (heading > M_PI) heading -= 2.0*M_PI;
    if (heading < -M_PI) heading += 2.0*M_PI;
    return heading;
}

// Compute the bid for the appropriate task
void compute_bid() 
{
    for (int k=0; k<NUM_TASKS; k++) {
        double distance = dist(active_tasks[k].task_x, active_tasks[k].task_y, my_pos[0], my_pos[1]);
        double completion_time = 0;
        //printf("Task %d is of type %d and robot %d is of type %d.\n", k, active_tasks[k].task_type, robot_id, robot_type);
        if (active_tasks[k].task_type == robot_type) {
            if (robot_type == 0) completion_time = 3;
            else completion_time = 1;
        } else {
            if (robot_type == 0) completion_time = 5;
            else completion_time = 9;
        }
        //printf("Robot %d needs %d to complete task %d.\n", robot_id, completion_time, k);
        //printf("Task %d is at position (%.4f, %.4f).\n", k, active_tasks[k].task_x, active_tasks[k].task_y);
        //printf("Robot %d is at position (%.4f, %.4f).\n", robot_id, my_pos[0], my_pos[1]);
        //printf("Robot %d is at a distance of %.4f from task %d.\n", robot_id, distance, k);
        active_tasks[k].bid_values[robot_id] = distance/MAX_VELOCITY + completion_time;
        //printf("Robot %d has a bid of %d for task %d.\n", robot_id, active_tasks[k].bid_values[robot_id], k);
    }
}

void print_task_table() 
{
    // Print the table header
    printf("--------------------------------------------------------------------------------\n");
    printf("| Task ID | Task Type |      Bid Values       | Best Bid | Best Bid Robot | Done |\n");
    printf("--------------------------------------------------------------------------------\n");

    // Print each task as a row in the table
    for (int i = 0; i < NUM_TASKS; i++) {
        printf("|   %3d   | %d | ", active_tasks[i].task_id, active_tasks[i].task_type);

        // Print bid values
        for (int j = 0; j < NUM_ROBOTS; j++) {
            if (active_tasks[i].bid_values[j] == INF) {
                printf("inf ");
            } else {
                printf("%.2f ", active_tasks[i].bid_values[j]);
            }
        }

        // Print best bid, best bid robot, and completion status
        printf("| %.4f |     %4d      | %d |\n",
               (active_tasks[i].best_bid_value == INF ? 1000 : active_tasks[i].best_bid_value),
               active_tasks[i].best_bid_robot,
               active_tasks[i].is_completed);
    }

    // Print table footer
    printf("--------------------------------------------------------------------------------\n");
}

void print_all_tasks() 
{
    printf("---------------------------------------------------------------------------\n");
    printf("| Task ID | Task Type |   X Coord   |   Y Coord   | Completed | In Active |\n");
    printf("---------------------------------------------------------------------------\n");

    for (int i = 0; i < 99; i++) { // Adjust size to match your `all_tasks` array length
        // Stop printing at the first empty component
        if (all_tasks[i].task_id == INF) {
            break;
        }

        printf("|   %3d   |     %1d     |   %8.2f |   %8.2f |     %1d     | %1d |\n",
               all_tasks[i].task_id,
               all_tasks[i].task_type,
               all_tasks[i].task_x,
               all_tasks[i].task_y,
               all_tasks[i].is_completed,
               all_tasks[i].in_active);
    }

    printf("--------------------------------------------------------------------\n");
}

void print_active_tasks() 
{
    printf("---------------------------------------------------------------\n");
    printf("| Task ID | Task Type |   X Coord   |   Y Coord   | Completed |\n");
    printf("---------------------------------------------------------------\n");

    for (int i = 0; i < NUM_TASKS; i++) { // Adjust size to match your `all_tasks` array length
        // Stop printing at the first empty component
        if (active_tasks[i].task_id == INF) {
            break;
        }

        printf("|   %3d   |     %1d     |   %8.2f |   %8.2f |     %1d    |\n",
               active_tasks[i].task_id,
               active_tasks[i].task_type,
               active_tasks[i].task_x,
               active_tasks[i].task_y,
               active_tasks[i].is_completed);
    }

    printf("--------------------------------------------------------------------\n");
}

void print_task(active_task_t task)
{
    printf("--------------------------------------------------------------------\n");
    printf("| Task ID | Task Type |   X Coord   |   Y Coord   | Best bid value |\n");
    printf("--------------------------------------------------------------------\n");

    printf("|   %3d   |     %1d     |   %8.2f |   %8.2f |     %.2f    |\n",
            task.task_id,
            task.task_type,
            task.task_x,
            task.task_y,
            task.best_bid_value);

    printf("--------------------------------------------------------------------\n");
}

int find_first_new_task_index()
{
    for (int i=0; i<99; i++) {
        if ((all_tasks[i].is_completed == 0) && (all_tasks[i].in_active == 0)) {
            return i;
        }
    }
    return INVALID;
}

void first_update_active_tasks()
{
   for (int j=0; j<NUM_TASKS; j++) {
        active_tasks[j].task_id = all_tasks[j].task_id;
        active_tasks[j].task_x = all_tasks[j].task_x;
        active_tasks[j].task_y = all_tasks[j].task_y;
        active_tasks[j].task_type = all_tasks[j].task_type;
        active_tasks[j].is_completed = all_tasks[j].is_completed;
        active_tasks[j].best_bid_robot = INF;
        active_tasks[j].best_bid_value = INFINITY;
        all_tasks[j].in_active = 1;
    }
    printf("Active tasks successfully initialized\n");
}

void update_active_tasks()
{
    for (int j=0; j<99; j++) {
        all_tasks[j].in_active = 0;
    }
    int k=0;
    while (k<10) {
        for (int j=0; j<99; j++) {
            if ((all_tasks[j].is_completed == 0) && (all_tasks[j].in_active == 0)) {
                active_tasks[k].task_id = all_tasks[j].task_id;
                active_tasks[k].task_x = all_tasks[j].task_x;
                active_tasks[k].task_y = all_tasks[j].task_y;
                active_tasks[k].task_type = all_tasks[j].task_type;
                active_tasks[k].is_completed = all_tasks[j].is_completed;
                active_tasks[k].best_bid_robot = INF;
                active_tasks[k].best_bid_value = INFINITY;
                all_tasks[j].in_active = 1;
                break;
            }
        }
        k += 1;
    }
}

// Update the list of active tasks
void update_new_active_task(int new_task_index) {
    for (int j=0; j<NUM_TASKS; j++) {
        if (active_tasks[j].is_completed == 1) {
            active_tasks[j].task_id = all_tasks[new_task_index].task_id;
            active_tasks[j].task_x = all_tasks[new_task_index].task_x;
            active_tasks[j].task_y = all_tasks[new_task_index].task_y;
            active_tasks[j].task_type = all_tasks[new_task_index].task_type;
            active_tasks[j].is_completed = all_tasks[new_task_index].is_completed;
            active_tasks[j].best_bid_robot = INF;
            active_tasks[j].best_bid_value = INFINITY;
        }
    }
    all_tasks[new_task_index].in_active = 1;
}

// Check if we received a message and extract information
static void receive_updates() 
{
   //printf("Robot %d receiving updates now.\n", robot_id);
    message_t msg;
    task_update_message_t task_msg;
    //int target_list_length = 0;
    //int i;
    //printf("There are %d messages in the queue for robot %d.\n", wb_receiver_get_queue_length(receiver_tag), robot_id);
    while (wb_receiver_get_queue_length(receiver_tag) > 0) {        
        /*
        printf("Raw message %d bytes: ", sizeof(message_t));
        for (size_t k = 0; k < sizeof(message_t); k++) {
            printf("%02X ", ((unsigned char*)pmsg)[k]);
        }
        printf("\n");
        */

        if (wb_receiver_get_data_size(receiver_tag) == sizeof(message_t)) {
            const message_t *pmsg = wb_receiver_get_data(receiver_tag);

            memcpy(&msg, pmsg, sizeof(message_t));
            wb_receiver_next_packet(receiver_tag);

            if (msg.event_state == MSG_QUIT) {
                wb_motor_set_velocity(left_motor, 0);
                wb_motor_set_velocity(right_motor, 0);
                wb_robot_step(TIME_STEP);
                exit(0);
            }

            if(msg.robot_id != robot_id) {
                fprintf(stderr, "Invalid message: robot_id %d "  "doesn't match receiver %d\n", msg.robot_id, robot_id);
                exit(1);
            }
            
            all_tasks[msg.event_id].task_id = msg.event_id;
            all_tasks[msg.event_id].task_x = msg.event_x;
            all_tasks[msg.event_id].task_y = msg.event_y;
            all_tasks[msg.event_id].task_type = msg.event_type; 
            all_tasks[msg.event_id].is_completed = msg.event_finished;  
        } else if (wb_receiver_get_data_size(receiver_tag) == sizeof(task_update_message_t)) {
            const task_update_message_t *tmsg = wb_receiver_get_data(receiver_tag);

            memcpy(&task_msg, tmsg, sizeof(task_update_message_t));
            wb_receiver_next_packet(receiver_tag);

            all_tasks[task_msg.finished_task_id].is_completed = 1;
            for (int i=0; i<NUM_TASKS; i++) {
                if (active_tasks[i].task_id == task_msg.finished_task_id) {
                    active_tasks[i].is_completed = 1;
                }
            }
            all_tasks[task_msg.new_task_id].in_active = 0;
            // if (all_tasks[task_msg.finished_task_id].is_completed) {
            //     printf("Task state was changed successfully.\n");
            // }
            // printf("Receiving info from the supervisor that task %d was finished.\n", task_msg.finished_task_id);
            //update_new_active_task(task_msg.new_task_id);
        }
    }

    update_active_tasks();
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* RESET and INIT (combined in function reset()) */
void reset(void) 
{
    wb_robot_init();
    int i;

    //get motors
    left_motor = wb_robot_get_device("left wheel motor");
    right_motor = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);

    // Enable GNSS localization
    gps = wb_robot_get_device("gps");
    wb_gps_enable(gps, TIME_STEP);
  
    char s[4] = "ps0";
    for(i=0; i<NB_SENSORS;i++) 
    {
        // the device name is specified in the world file
        ds[i]=wb_robot_get_device(s);      
        s[2]++; // increases the device number
        wb_distance_sensor_enable(ds[i],64);
    } 

    clock = 0;
    indx = 0;
    
    // Init target positions to "INVALID"
    for(int i=0; i<99; i++){ 
        all_tasks[i].task_x = 0;
        all_tasks[i].task_y = 0;
        all_tasks[i].task_id = INF;
        all_tasks[i].task_type = INVALID;
        all_tasks[i].is_completed = INVALID;
        all_tasks[i].in_active = INVALID;
    }

    // Init active tas positions to "INVALID"
    for (int i=0; i<NUM_TASKS; i++) {
        active_tasks[i].task_id = INVALID;
        active_tasks[i].task_x = INVALID;
        active_tasks[i].task_y = INVALID;
        active_tasks[i].task_type = INVALID;
        active_tasks[i].is_completed = INVALID;
        active_tasks[i].best_bid_value = INVALID;
        active_tasks[i].best_bid_robot = INVALID;
        for (int j=0; j<NUM_ROBOTS; j++) {
            active_tasks[i].bid_values[j] = INF;
        }
    }

    // Start in the DEFAULT_STATE
    state = DEFAULT_STATE;

    // read robot id and state from the robot's name
    char* robot_name; 
    robot_name = (char*) wb_robot_get_name();
    int tmp_id;
    if (sscanf(robot_name, "e-puck%d", &tmp_id)) {robot_id = (uint16_t)tmp_id;} 
    else {fprintf(stderr, "ERROR: couldn't parse my id %s \n", robot_name); exit(1);}

    // What type of robot am I ?
    if ((robot_id == 0) || (robot_id == 1)) {robot_type = 0;}
    else robot_type = 1;
    printf("Robot %d is of type %d\n", robot_id, robot_type);

    // Am I used in this simulation?
    if (robot_id >= NUM_ROBOTS) {
        fprintf(stderr, "Robot %d is not needed. exiting ...\n", robot_id); 
        wb_robot_cleanup(); 
        exit(0);
    }

    // Initialize the emitters and receivers for communication with the supervisor
    emitter_tag = wb_robot_get_device("emitter");
    wb_emitter_set_range(emitter_tag, 2.0); 
    receiver_tag = wb_robot_get_device("receiver");
    wb_receiver_enable(receiver_tag, RX_PERIOD); // listen to incoming data every 1000ms
    wb_receiver_set_channel(receiver_tag, robot_id+1);
    //printf("The receiver channel for robot %d is number %d.\n", robot_id, wb_receiver_get_channel(receiver_tag));
    
    // Initialize the emitters and receiver for short communication (inter robot communication)
    sr_emitter_tag = wb_robot_get_device("em_short_range");
    wb_emitter_set_range(sr_emitter_tag, SR_COM_RANGE);
    sr_receiver_tag = wb_robot_get_device("rc_short_range");
    wb_receiver_enable(sr_receiver_tag, RX_PERIOD);
    wb_receiver_set_channel(sr_receiver_tag, robot_id+NUM_ROBOTS+1);

    my_pos[0] = 0;
    my_pos[1] = 0;
    my_heading = 0;
    
    // Seed random generator
    srand(getpid());

    // Reset stats
    stat_max_velocity = 0.0;
}

void build_message(active_task_t task, robot_robot_message_t* msg)
{
    msg->robot_id = robot_id;
    msg->task_id = task.task_id;
    for (int j=0; j<NUM_ROBOTS; j++) {
        msg->bid_values[j] = task.bid_values[j];
    }
    msg->is_completed = task.is_completed;
}

void send_task_info() {
    static int last_broadcast_time = 0;
    if (clock - last_broadcast_time < 200) return; // Broadcast every 200ms

    robot_robot_message_t msg;
    for (int i = 0; i < NUM_ROBOTS; i++) {
        if (i != robot_id) {
            while (wb_emitter_get_channel(sr_emitter_tag) != i + NUM_ROBOTS + 1) {
                wb_emitter_set_channel(sr_emitter_tag, i + NUM_ROBOTS + 1);
            }
            for (int j = 0; j < NUM_TASKS; j++) {
                build_message(active_tasks[j], &msg);
                wb_emitter_send(sr_emitter_tag, &msg, sizeof(robot_robot_message_t));
            }
        }
    }

    last_broadcast_time = clock;
}

void receive_task_info() {
    robot_robot_message_t msg;
    int processed_messages = 0;
    
    while (wb_receiver_get_queue_length(sr_receiver_tag) > 0 && processed_messages < 10) { // Limit messages processed
        processed_messages++;
        
        if (wb_receiver_get_data_size(sr_receiver_tag) != sizeof(robot_robot_message_t)) {
            printf("Error: Unexpected message size. Skipping packet.\n");
            wb_receiver_next_packet(sr_receiver_tag);
            continue;
        }

        const message_t *rmsg = wb_receiver_get_data(sr_receiver_tag);
        memcpy(&msg, rmsg, sizeof(robot_robot_message_t));
        wb_receiver_next_packet(sr_receiver_tag);

        if (msg.task_id < 0 || msg.task_id >= 99) {
            printf("Error: Received invalid task ID %d.\n", msg.task_id);
            continue;
        }

        for (int j = 0; j < NUM_ROBOTS; j++) {
            if (msg.bid_values[j] != INF) {
                for (int k = 0; k < NUM_TASKS; k++) {
                    if (all_tasks[msg.task_id].task_id == active_tasks[k].task_id) {
                        active_tasks[k].bid_values[j] = msg.bid_values[j];
                    }
                }
            }
        }
    }
}

// Compare the bids of the robots on every task
void compare_bids()
{
    for (int i = 0; i < NUM_TASKS; i++) {
        if (active_tasks[i].task_id == INVALID) {
            continue; // Skip invalid tasks
        }
        // Find the robot with the best bid
        for (int j = 0; j < NUM_ROBOTS; j++) {
            if (j == 0 || active_tasks[i].bid_values[j] < active_tasks[i].bid_values[active_tasks[i].best_bid_robot]) {
                active_tasks[i].best_bid_robot = j;
                active_tasks[i].best_bid_value = active_tasks[i].bid_values[j];
            }
        }

        // Print the best bid details
        //printf("Best Bid: Robot %d with Bid = %.2f\n\n", 
        //       active_tasks[i].best_bid_robot, 
        //       active_tasks[i].best_bid_value);
    }
}

// Claim a task if I have the best bid
task_t claim_task()
{
    task_t goal_task = { .task_id = INVALID, .task_x = 0, .task_y = 0, .task_type = INVALID, .is_completed = 0 };
    double lowest_bid = INF;
    int best_bid_counter = 0; // Count on how many tasks do I have the best bid
    for (int i=0; i<NUM_TASKS; i++) {
        if (active_tasks[i].is_completed) {
            continue;
        }
        if (active_tasks[i].best_bid_robot == robot_id) {
            best_bid_counter += 1;
            if (active_tasks[i].bid_values[robot_id] < lowest_bid) {
                lowest_bid = active_tasks[i].bid_values[robot_id];
                goal_task.task_id = active_tasks[i].task_id;
                goal_task.task_x = active_tasks[i].task_x;
                goal_task.task_y = active_tasks[i].task_y;
                goal_task.task_type = active_tasks[i].task_type;
                goal_task.is_completed = 0;
                target_valid = 1;
            }
        }
    }
    if (best_bid_counter == 0) {
        target_valid = 0;
        //printf("Robot %d does not have the best bid on any task.\n", robot_id);
    } else {
        //printf("Robot %d going towards task %d at (%.4f, %.4f).\n", robot_id, goal_task.task_id, goal_task.task_x, goal_task.task_y);
    }
    return goal_task;
} 

void update_finished_task_state(task_t task)
{
    all_tasks[task.task_id].is_completed = 1;
    task.is_completed = 1;
    target_valid = 0;
    for (int i=0; i<NUM_TASKS; i++) {
        if (active_tasks[i].task_id == task.task_id) {
            active_tasks[i].is_completed = 1;
        }
    }
}

void update_state(int _sum_distances, task_t task, int handling_time)
{
    static int handle_time_counter = 0;
    //printf("The sum of the sensors of robot %d is equal to %d\n", robot_id, _sum_distances); 
    //if (target_valid) {printf("Robot %d has a valid target.\n", robot_id);}
    //if (_sum_distances < STATECHANGE_DIST) { printf("Robot %d does not detect any obstacle.\n", robot_id);}
    //printf("Robot %d is %.4f from task %d\n", robot_id, dist(task.task_x, task.task_y, my_pos[0], my_pos[1]), task.task_id);
    if (state == GO_TO_GOAL && _sum_distances > STATECHANGE_DIST){
        state = OBSTACLE_AVOID;
    }
    else if (state == OBSTACLE_AVOID && _sum_distances > STATECHANGE_DIST) {
        state = OBSTACLE_AVOID;
    }
    else if(state == OBSTACLE_AVOID && _sum_distances < STATECHANGE_DIST && target_valid){
        state = GO_TO_GOAL;
    }
    else if(state == STAY && target_valid){
        state = GO_TO_GOAL;
    }
    else if (state == GO_TO_GOAL && dist(task.task_x, task.task_y, my_pos[0], my_pos[1]) < 0.1) {
        state = HANDLING_TASK;
    } 
    else if (state == FINISHED_TASK) {
        state = STAY;
    }
    else if (state == HANDLING_TASK && handle_time_counter < handling_time) {
        state = HANDLING_TASK;
        handle_time_counter += TIME_STEP;
    } else if (state == HANDLING_TASK && handle_time_counter > handling_time) {
        state = FINISHED_TASK;
        printf("Finished the task in %d ms\n", handle_time_counter);
        handle_time_counter = 0;
        update_finished_task_state(task);
    }
    else if (state == GO_TO_GOAL && !target_valid) {
        state = STAY;
    }
}

// Odometry
void update_self_motion(int msl, int msr) {
    double theta = my_heading;
  
    // Compute deltas of the robot
    double dr = (double)msr * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
    double dl = (double)msl * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
    double du = (dr + dl)/2.0;
    double dtheta = (dr - dl)/AXLE_LENGTH;
  
    // Compute deltas in the environment
    double dx = du * cosf(theta);
    double dy = du * sinf(theta);
  
    // Update position
    my_pos[0] += dx;
    my_pos[1] -= dy;
    my_heading -= dtheta;
    
    // Keep orientation within 0, 2pi
    if (my_heading > M_PI) my_heading -= 2.0*M_PI;
    if (my_heading < -M_PI) my_heading += 2.0*M_PI;

    // Keep track of highest velocity for modelling
    double velocity = du * 1000.0 / (double) TIME_STEP;
    if (state == GO_TO_GOAL && velocity > stat_max_velocity)
        stat_max_velocity = velocity;
}

// Compute wheel speed to avoid obstacles
void compute_avoid_obstacle(int *msl, int *msr, int distances[]) 
{
    int d1=0,d2=0;       // motor speed 1 and 2     
    int sensor_nb;       // FOR-loop counters    

    for(sensor_nb=0;sensor_nb<NB_SENSORS;sensor_nb++)
    {   
       d1 += (distances[sensor_nb]-300) * Interconn[sensor_nb];
       d2 += (distances[sensor_nb]-300) * Interconn[sensor_nb + NB_SENSORS];
    }
    d1 /= 80; d2 /= 80;  // Normalizing speeds

    *msr = d1+BIAS_SPEED; 
    *msl = d2+BIAS_SPEED; 
    limit(msl,MAX_SPEED);
    limit(msr,MAX_SPEED);
}

void astolfi_control(task_t goal_task, int *msl, int *msr) 
{
    float Kp = 15;  // > 0
    float Ka = 45;  // > Kp
    //float Kb = -5;  // < 0

    double delta_x = goal_task.task_x - my_pos[0];
    double delta_y = goal_task.task_y - my_pos[1];
    double reference_angle = atan2(delta_y, delta_x);

    //double rho = sqrt(delta_x * delta_x + delta_y * delta_y);
    double alpha = reference_angle - my_heading;

    // Keep orientation within [-π, π]
    if (alpha > M_PI) alpha -= 2.0 * M_PI;
    if (alpha < -M_PI) alpha += 2.0 * M_PI;

    //double beta = - my_heading - alpha;

    double v = Kp; // Linear velocity
    double omega = Ka * alpha ;//+ Kb * beta; // Angular velocity

    // Convert to wheel speeds
    *msr = (int)((AXLE_LENGTH / 2) * omega + v) / WHEEL_RADIUS;
    *msl = (int)(v - (AXLE_LENGTH / 2) * omega) / WHEEL_RADIUS;
}

// Compute wheel speeds to go towards a task
void compute_go_to_task(task_t task, int *msl, int *msr)
{
    // Compute vector to goal
    float a = task.task_x - my_pos[0];
    float b = task.task_y - my_pos[1];
    // Compute wanted position from task position and current location
    float x = a*cosf(my_heading) - b*sinf(my_heading);
    float y = a*sinf(my_heading) + b*cosf(my_heading);

    // Range and bearing
    float range = sqrtf(x*x + y*y);   // Distance to the wanted position
    float bearing = atan2(y, x);     // Orientation of the wanted position

    // Keep bearing between pi and -pi
    if (bearing > M_PI) bearing -= 2.0 * M_PI;
    if (bearing < -M_PI) bearing += 2.0 * M_PI;

    float Ku = 1.0;  // Higher gain for larger distances
    float Kw = 10.0; // Lower gain near the target

    // Compute forward control
    float u = Ku * range * cosf(bearing);
    // Compute rotational control
    float w = Kw * range * sinf(bearing);

    // Convert to wheel speeds
    *msl = (int)(50 * (u - AXLE_LENGTH * w / 2.0) / WHEEL_RADIUS);
    *msr = (int)(50 * (u + AXLE_LENGTH * w / 2.0) / WHEEL_RADIUS);

    // Limit speeds
    limit(msl, MAX_SPEED);
    limit(msr, MAX_SPEED);
}

void update_task_info_for_supervisor(task_t task)
{
    if (state == FINISHED_TASK) {
        task.is_completed = 1;

        robot_message_t msg;
        while(wb_emitter_get_channel(emitter_tag) != robot_id+1) {
            wb_emitter_set_channel(emitter_tag, robot_id+1);
        }
        //memset(&msg, 0, sizeof(robot_message_t));
        msg.robot_id = robot_id;
        msg.event_id = task.task_id;
        msg.finished = task.is_completed;
        wb_emitter_send(emitter_tag, &msg, sizeof(robot_message_t));
    }
}

// Function to swap two tasks
void swap(active_task_t *a, active_task_t *b) {
    active_task_t temp = *a;
    *a = *b;
    *b = temp;
}

double calculate_path_cost(active_task_t tasks[], int num_tasks) {
    double cost = 0.0;
    for (int i = 0; i < num_tasks; i++) {
        double completion_time = 0;
        if (tasks[i].task_type == robot_type) {
            if (robot_type == 0) completion_time = 3;
            else completion_time = 1;
        } else {
            if (robot_type == 0) completion_time = 5;
            else completion_time = 9;
        }

        if (i == 0) {
            cost += dist(tasks[i].task_x, tasks[0].task_y, my_pos[0], my_pos[1])/MAX_VELOCITY;
        } else {
            cost += dist(tasks[i].task_x, tasks[i].task_y, tasks[i-1].task_x, tasks[i-1].task_y)/MAX_VELOCITY;
        }
        cost += completion_time;
    }
    return cost;
}

void permute(active_task_t tasks[], int l, int r, active_task_t best_order[], double *min_cost) {
    if (l == r) {
        // Calculate the cost for the current permutation
        double cost = calculate_path_cost(tasks, r+1);

        // If this cost is the lowest, update the best order
        if (cost < *min_cost) {
            *min_cost = cost;
            for (int i = 0; i <= r; i++) {
                best_order[i] = tasks[i];
            }
        }
        return;
    }

    for (int i = l; i <= r; i++) {
        // Generate permutations by swapping
        swap(&tasks[l], &tasks[i]);
        permute(tasks, l + 1, r, best_order, min_cost);
        swap(&tasks[l], &tasks[i]); // Backtrack
    }
}

void compare_won_tasks(active_task_t won_tasks[], int num_won_tasks, active_task_t best_order[])
{
    double min_cost = INF;

    // Find the best order using permutations
    permute(won_tasks, 0, num_won_tasks - 1, best_order, &min_cost);

    // printf("Best order for tasks: ");
    // for (int i = 0; i < num_won_tasks; i++) {
    //     printf("%d ", best_order[i].task_id);
    // }
    // printf("with cost: %.2f\n", min_cost);
}

task_t three_step_plan()
{
    task_t goal_task = { .task_id = INVALID, .task_x = 0, .task_y = 0, .task_type = INVALID, .is_completed = 0 };
    active_task_t won_tasks[NUM_TASKS];
    // Initialize to known values to prevent crashes
    for (int i=0; i<NUM_TASKS; i++) {
        won_tasks[i].task_id = INVALID;
        won_tasks[i].task_x = 0;
        won_tasks[i].task_y = 0;
        won_tasks[i].task_type = INVALID;
        won_tasks[i].best_bid_robot = INVALID;
        won_tasks[i].best_bid_value = INF;
        for (int j=0; j<NUM_ROBOTS; j++) {
            won_tasks[i].bid_values[j] = INF;
        }
    }
    active_task_t best_order[3];
    for (int i=0; i<3; i++) {
        best_order[i].task_id = INVALID;
        best_order[i].task_x = 0;
        best_order[i].task_y = 0;   
        best_order[i].task_type = INVALID;
        best_order[i].best_bid_robot = INVALID;
        best_order[i].best_bid_value = INF;
        for (int j=0; j<NUM_ROBOTS; j++) {
            best_order[i].bid_values[j] = INF;
        }
    }
    int won_count = 0;
    
    // Collect tasks where this robot has the best bid
    for (int i = 0; i < NUM_TASKS; i++) {
        if (!active_tasks[i].is_completed && active_tasks[i].best_bid_robot == robot_id) {
            won_tasks[won_count] = active_tasks[i];
            //print_task(won_tasks[won_count]);
            won_count += 1;
        }
    }

    // Select up to 3 tasks with the lowest bids
    int num_selected = (won_count < 3) ? won_count : 3;
    active_task_t selected_tasks[3];

    if (won_count != 0) {
        for (int i = 0; i < num_selected; i++) {
            double dummy_bid = 100.0;
            int best_index = -1;

            for (int j = 0; j < won_count; j++) {
                if (i == 0) {
                    // Select the task with the smallest bid for the first iteration
                    if (won_tasks[j].best_bid_value < dummy_bid) {
                        best_index = j;
                        dummy_bid = won_tasks[j].best_bid_value;
                    }
                } else {
                    // Select the next smallest bid greater than the previous one
                    if (won_tasks[j].best_bid_value < dummy_bid &&
                        won_tasks[j].best_bid_value > selected_tasks[i - 1].best_bid_value) {
                        best_index = j;
                        dummy_bid = won_tasks[j].best_bid_value;
                        //print_task(won_tasks[j]);
                    }
                }
            }

            // Ensure a valid task was found
            if (best_index != -1) {
                selected_tasks[i] = won_tasks[best_index];
                sanity_check(&selected_tasks[i], "selected_tasks[i]");
            } else {
                // Handle edge case: fewer tasks than expected
                printf("No valid task found for position %d for robot %d.\n", i, robot_id);
                break;
            }
        }
    }

    switch (num_selected) {
        case 0:
            target_valid = 0;
            //printf("Robot %d does not have the best bid on any tasks.\n", robot_id);
            return goal_task;  

        case 1:
            target_valid = 1;
            goal_task.task_id = selected_tasks[0].task_id;
            goal_task.task_x = selected_tasks[0].task_x;
            goal_task.task_y = selected_tasks[0].task_y;
            goal_task.task_type = selected_tasks[0].task_type;
            //printf("Robot %d only has the best bid on task %d.\n", robot_id, goal_task.task_id);
            return goal_task;

        case 2:
            target_valid = 1;
            compare_won_tasks(selected_tasks, 2, best_order);
            goal_task.task_id = best_order[0].task_id;
            goal_task.task_x = best_order[0].task_x;
            goal_task.task_y = best_order[0].task_y;
            goal_task.task_type = best_order[0].task_type;
            return goal_task;

        case 3:
            target_valid = 1;
            compare_won_tasks(selected_tasks, 3, best_order);
            goal_task.task_id = best_order[0].task_id;
            goal_task.task_x = best_order[0].task_x;
            goal_task.task_y = best_order[0].task_y;
            goal_task.task_type = best_order[0].task_type;
            return goal_task;

        default:
            target_valid = 0;
            return goal_task;
    }
    target_valid = 0;
    return goal_task;
}

// RUN e-puck
void run(int ms)
{
    float msl_w, msr_w;
    // Motor speed and sensor variables	
    int msl=0,msr=0;                // motor speed left and right
    int distances[NB_SENSORS];  // array keeping the distance sensor readings
    int sum_distances=0;        // sum of all distance sensor inputs, used as threshold for state change.  	

    static double prev_pos[2] = {0.0, 0.0};  // Store previous position
    static double prev_heading = 0.0;       // Store previous heading 	

    static int handling_time = 0;           // Store the handling time of a task
    static double active_time = 0;             // Store the time a robot is active for

    static int active_task_init = 0;

    // Other variables
    int sensor_nb;

    // Add the weighted sensors values
    for(sensor_nb=0;sensor_nb<NB_SENSORS;sensor_nb++)
    {  
        distances[sensor_nb] = wb_distance_sensor_get_value(ds[sensor_nb]);
        sum_distances += distances[sensor_nb];
    }

    // Get info from supervisor
    receive_updates();

    if (active_task_init <= 2) {
        first_update_active_tasks();
        active_task_init += 1;
    }

    /* get current position */
    const double *gps_pos = wb_gps_get_values(gps);
    my_pos[0] = gps_pos[0];
    my_pos[1] = gps_pos[1];
    /* get current heading */
    if (clock > 2*TIME_STEP) {
        my_heading = compute_heading(my_pos, prev_pos, prev_heading);
    } else {
        my_heading = 0;
    }
    //printf("Robot %d previously at (%.6f, %.6f, %.6f).\n", robot_id, prev_pos[0], prev_pos[1], 180*prev_heading/M_PI);
    //printf("Robot %d at (%.6f, %.6f, %.6f).\n", robot_id, my_pos[0], my_pos[1], 180*my_heading/M_PI);  

    compute_bid();
    
    if (clock > 2*TIME_STEP) {
        send_task_info();

        receive_task_info();
    }

    compare_bids();

    task_t goal_task = three_step_plan();

    //task_t goal_task = claim_task();

    //printf("Robot %d currently at (%.4f, %.4f).\n", robot_id, my_pos[0], my_pos[1]);
    
    if (robot_type == goal_task.task_type) {
        if (robot_type == 0) handling_time = 3000;
        else handling_time = 1000;
    } else {
        if (robot_type == 0) handling_time = 5000;
        else handling_time = 9000;
    } 

    update_state(sum_distances, goal_task, handling_time);

    //printf("Robot %d currently in state %d.\n", robot_id, state);

    // Set wheel speeds depending on state
    switch (state) {
        case STAY:
            msl = 0;
            msr = 0;
            break;

        case HANDLING_TASK:
            msl = 0;
            msr = 0;
            break;

        case GO_TO_GOAL:
            //compute_go_to_task(goal_task, &msl, &msr);
            //compute_go_to_task_braitenberg(goal_task, &msl, &msr);
            astolfi_control(goal_task, &msl, &msr);
            break;

        case OBSTACLE_AVOID:
            compute_avoid_obstacle(&msl, &msr, distances);
            break;

        case RANDOM_WALK:
            msl = 400;
            msr = 400;
            break;

        case FINISHED_TASK:
            msl = 0;
            msr = 0;
            break;

        default:
            printf("Invalid state: robot_id %d \n", robot_id);
    }

    // Set the speed
    msl_w = msl*MAX_SPEED_WEB/1000;
    msr_w = msr*MAX_SPEED_WEB/1000;
    wb_motor_set_velocity(left_motor, msl_w);
    wb_motor_set_velocity(right_motor, msr_w);

    if (state != STAY) {
        active_time += (double) TIME_STEP/1000;
        //printf("Robot %d active for %.2f s\n", robot_id, active_time);
    }

    //printf("Robot %d active time : %d", robot_id, active_time);
    if (active_time >= 120) {
        wb_motor_set_velocity(left_motor, 0);
        wb_motor_set_velocity(right_motor, 0);  
        wb_robot_step(TIME_STEP);
        printf("Robot %d has no energy left.\n", robot_id);
        exit(0);
    }

    // Update the previous pose
    prev_pos[0] = my_pos[0];
    prev_pos[1] = my_pos[1];
    prev_heading = my_heading;

    update_task_info_for_supervisor(goal_task);

    // Print the task table summary
    //print_task_table();

    //print_all_tasks();

    // Update clock
    clock += ms;
}

// MAIN
int main(int argc, char **argv) 
{
    reset();
    // RUN THE MAIN ALGORIHM
    while (wb_robot_step(TIME_STEP) != -1) {
        //printf("ROBOT %d LOOP\n", robot_id);
        run(TIME_STEP);
    }
    wb_robot_cleanup();

    return 0;
}
