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
#define MAX_SPEED         800     // Maximum speed
#define MAX_VELOCITY      0.5 

#define INVALID          -999
#define BREAK            -999 //for physics plugin
#define INF               1000

#define NUM_ROBOTS 5 // Change this also in the supervisor!
#define NUM_TASKS 10
#define SR_COM_RANGE 0.3


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
int lmsg, rmsg;             // Communication variables
int indx;                   // Event index to be sent to the supervisor

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

int find_last_non_empty_task_id() {
    for (int i = 98; i >= 0; i--) { // Start from the last index
        if (all_tasks[i].task_id != INVALID && all_tasks[i].is_completed == 0) {
            return all_tasks[i].task_id; // Return the task ID of the first valid task found
        }
    }
    return -1; // Return -1 if no valid task is found
}

double compute_heading(const double *pos, double *prev_pos, double prev_heading) {
    double x = 0, y = 0;

    if ((prev_pos[0] != 0) && (prev_pos[1] != 0)) {
        x = pos[0] - prev_pos[0];
        y = pos[1] - prev_pos[1];
    }

    if ((x == 0) && (y == 0)) return prev_heading;

    double heading =  atan2(y, x);
    if (heading > 2*M_PI) heading -= 2.0*M_PI;
    if (heading < 0) heading += 2.0*M_PI;
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

void print_task_table() {
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

// Update the list of active tasks
void update_active_tasks() {
    for (int j=0; j<NUM_TASKS; j++) {
        if (active_tasks[j].is_completed == INVALID) {
            active_tasks[j].task_id = all_tasks[j].task_id;
            active_tasks[j].task_x = all_tasks[j].task_x;
            active_tasks[j].task_y = all_tasks[j].task_y;
            active_tasks[j].task_type = all_tasks[j].task_type;
            active_tasks[j].is_completed = all_tasks[j].is_completed;
            active_tasks[j].best_bid_robot = INF;
            active_tasks[j].best_bid_value = INFINITY;
        } else if (active_tasks[j].is_completed == 1) {
            int new_task_index = find_last_non_empty_task_id();
            active_tasks[j].task_id = all_tasks[new_task_index].task_id;
            active_tasks[j].task_x = all_tasks[new_task_index].task_x;
            active_tasks[j].task_y = all_tasks[new_task_index].task_y;
            active_tasks[j].task_type = all_tasks[new_task_index].task_type;
            active_tasks[j].is_completed = all_tasks[new_task_index].is_completed;
            active_tasks[j].best_bid_robot = INF;
            active_tasks[j].best_bid_value = INFINITY;
        }
    }
}


// Check if we received a message and extract information
static void receive_updates() 
{
   //printf("Robot %d receiving updates now.\n", robot_id);
    message_t msg;
    int target_list_length = 0;
    //int i;
    //printf("There are %d messages in the queue for robot %d.\n", wb_receiver_get_queue_length(receiver_tag), robot_id);
    while (wb_receiver_get_queue_length(receiver_tag) > 0) {

        const message_t *pmsg = wb_receiver_get_data(receiver_tag);
        //assert(wb_receiver_get_data_size(receiver_tag) == sizeof(message_t));
        
        /*
        printf("Raw message %d bytes: ", sizeof(message_t));
        for (size_t k = 0; k < sizeof(message_t); k++) {
            printf("%02X ", ((unsigned char*)pmsg)[k]);
        }
        printf("\n");
        */

        // save a copy, cause wb_receiver_next_packet invalidates the pointer
        memcpy(&msg, pmsg, sizeof(message_t));
        wb_receiver_next_packet(receiver_tag);
        
        // double check this message is for me
        // communication should be on specific channel per robot
        // channel = robot_id + 1, channel 0 reserved for physics plguin
        if(msg.robot_id != robot_id) {
            fprintf(stderr, "Invalid message: robot_id %d "  "doesn't match receiver %d\n", msg.robot_id, robot_id);
            //return;
            exit(1);
        }
        
        all_tasks[msg.event_id].task_id = msg.event_id;
        all_tasks[msg.event_id].task_x = msg.event_x;
        all_tasks[msg.event_id].task_y = msg.event_y;
        all_tasks[msg.event_id].task_type = msg.event_type; 
        all_tasks[msg.event_id].is_completed = msg.event_finished;
    }

    // Find target list length
    for (int i=0; i<99; i++) {
        if (all_tasks[i].is_completed == 0) {
            target_list_length += 1;
        }
    }
    //printf("TARGET LIST LENGTH : %d\n", target_list_length);
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
        all_tasks[i].task_id = INVALID;
        all_tasks[i].task_type = INVALID;
        all_tasks[i].is_completed = INVALID;
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

// Send task information to every robot in range that is not me
void send_task_info()
{
    robot_robot_message_t msg;
    for (int i=0; i<NUM_ROBOTS; i++) {
        if (i != robot_id) {
            while (wb_emitter_get_channel(sr_emitter_tag) != i+NUM_ROBOTS+1) {
                wb_emitter_set_channel(sr_emitter_tag, i+NUM_ROBOTS+1);
            }
            for (int j=0; j<NUM_TASKS; j++) {
                build_message(active_tasks[j], &msg);
                wb_emitter_send(sr_emitter_tag, &msg, sizeof(robot_robot_message_t));
            }
        }
    }
}

// Receive task information from every robot in range that is not me
void receive_task_info() {
    robot_robot_message_t msg;
    //printf("There are %d messages in the queue for robot %d.\n", wb_receiver_get_queue_length(sr_receiver_tag), robot_id);
    while (wb_receiver_get_queue_length(sr_receiver_tag) > 0) {
        if (wb_receiver_get_data_size(sr_receiver_tag) != sizeof(robot_robot_message_t)) {
            //printf("Error: Message size mismatch for robot %d. Expected %lu, got %d.\n",
            //    robot_id, sizeof(robot_robot_message_t), wb_receiver_get_data_size(sr_receiver_tag));
            wb_receiver_next_packet(sr_receiver_tag);  // Skip this packet
            continue;
        }
        const message_t *rmsg = wb_receiver_get_data(sr_receiver_tag);

        // Make a copy of the message
        memcpy(&msg, rmsg, sizeof(robot_robot_message_t));
        wb_receiver_next_packet(sr_receiver_tag);

        //printf("Message received : task id=%d, robot id=%d, bid value=%d.\n", msg->task_id, msg->robot_id, msg->bid_value);
        for (int j = 0; j < NUM_ROBOTS; j++) {
            if (msg.bid_values[j] != INF) {  // Valid bid value
                // Update only if the new bid is better (lower value)
                //if (active_tasks[msg.task_id].bid_values[j] == INF ||
                //    msg.bid_values[j] < active_tasks[msg.task_id].bid_values[j]) {
                    active_tasks[msg.task_id].bid_values[j] = msg.bid_values[j];
                    //printf("Updated Task %d: Robot %d bid value %.2f.\n",
                    //       msg->task_id, j, msg->bid_values[j]);
                //}
            }
            all_tasks[msg.task_id].is_completed = msg.is_completed;
        }
    }  
    update_active_tasks;
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
    if (my_heading > 2*M_PI) my_heading -= 2.0*M_PI;
    if (my_heading < 0) my_heading += 2.0*M_PI;

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
    float Kp = 10;  // > 0
    float Ka = 20;  // > Kp

    double delta_x = goal_task.task_x - my_pos[0];
    double delta_y = goal_task.task_y - my_pos[1];
    double reference_angle = atan2(delta_x, delta_y);

    double rho = sqrt(delta_x*delta_x + delta_y*delta_y);
    double alpha = reference_angle - my_heading;

    // Keep orientation within 0, 2pi
    if (alpha > 2*M_PI) alpha -= 2.0*M_PI;
    if (alpha < 0) alpha += 2.0*M_PI;

    double power = 0.25;
    //double K_power = 12 / (pow(goal_radius, power));
    double v = Kp;//* K_power * pow(rho, power);
    double omega = Ka * alpha;

    *msr = (int) ((AXLE_LENGTH/2) * omega + v) / WHEEL_RADIUS;
    *msl = (int) (v - (AXLE_LENGTH/2) * omega) / WHEEL_RADIUS;
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

    //float Ku = 0.5 ;     // Forward control gain
    //float Kw = 20.0;    // Rotation control gain
    //float range = 1;

    // Range and bearing
    float range = sqrtf(x*x + y*y);   // Distance to the wanted position
    float bearing = atan2(y, x);     // Orientation of the wanted position

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

    /* get current position */
    const double *gps_pos = wb_gps_get_values(gps);
    my_pos[0] = gps_pos[0];
    my_pos[1] = gps_pos[1];
    /* get current heading */
    my_heading = compute_heading(my_pos, prev_pos, prev_heading);
    //printf("Robot %d previously at (%.6f, %.6f, %.6f).\n", robot_id, prev_pos[0], prev_pos[1], prev_heading);
    //printf("Robot %d at (%.6f, %.6f, %.6f).\n", robot_id, my_pos[0], my_pos[1], my_heading);  

    compute_bid();
    
    if (clock > 2*TIME_STEP) {
        send_task_info();

        receive_task_info();
    }

    compare_bids();

    task_t goal_task = claim_task();

    //printf("Robot %d currently at (%.4f, %.4f).\n", robot_id, my_pos[0], my_pos[1]);
    
    if (robot_type == goal_task.task_type) {
        if (robot_type == 0) handling_time = 3000;
        else handling_time = 1000;
    } else {
        if (robot_type == 0) handling_time = 5000;
        else handling_time = 9000;
    } 

    update_state(sum_distances, goal_task, handling_time);

    printf("Robot %d currently in state %d.\n", robot_id, state);

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
    update_self_motion(msl, msr);

    // Update the previous pose
    prev_pos[0] = my_pos[0];
    prev_pos[1] = my_pos[1];
    prev_heading = my_heading;

    update_task_info_for_supervisor(goal_task);

    // Print the task table summary
    print_task_table();

    // Update clock
    clock += ms;
}

// MAIN
int main(int argc, char **argv) 
{
    reset();
    // RUN THE MAIN ALGORIHM
    while (wb_robot_step(TIME_STEP) != -1) {
        printf("ROBOT LOOP\n");
        run(TIME_STEP);
    }
    wb_robot_cleanup();

    return 0;
}
