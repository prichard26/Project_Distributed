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

#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/distance_sensor.h>
#include <webots/radio.h>
#include <webots/motor.h>
  
#include <webots/supervisor.h> 

#include "../auct_super/message.h"

#define MAX_SPEED_WEB      6.28    // Maximum speed webots
WbDeviceTag left_motor; //handler for left wheel of the robot
WbDeviceTag right_motor; //handler for the right wheel of the robot


#define DEBUG 1
#define TIME_STEP           64                  // Timestep (ms)
#define RX_PERIOD           2                   // time difference between two received elements (ms) (1000)

#define AXLE_LENGTH         0.052               // Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS     0.00628             // Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS        0.0205              // Wheel radius (meters)
#define DELTA_T             TIME_STEP/1000      // Timestep (seconds)
#define MAX_SPEED           800                 // Maximum speed
#define AVG_SPEED           0.01425             // in [m/s]

#define INVALID             -999
#define BREAK               -999                //for physics plugin

#define NUM_ROBOTS          5                   // Change this also in the supervisor!


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Collective decision parameters */

#define STATECHANGE_DIST  555  // minimum value of all sensor inputs combined to change to obstacle avoidance mode

typedef enum {
    STAY            = 1,
    GO_TO_GOAL      = 2,                    // Initial state aliases
    OBSTACLE_AVOID  = 3,
    RANDOM_WALK     = 4,
    HANDLING_TASK   = 5,
} robot_state_t;

typedef struct {
    float x;
    float y;
    Event_type type;
} event_data;

#define DEFAULT_STATE (STAY)

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* e-Puck parameters */

#define NB_SENSORS           8
#define BIAS_SPEED           400

// Weights for the Braitenberg algorithm
// NOTE: Weights from reynolds2.h
int Interconn[16] = {17,29,34,10,8,-38,-56,-76,-72,-58,-36,8,10,36,28,18};

double energy_level;       // Energy level of the robot
const double max_energy = 2 * 60 * 1000; // 2 minutes in milliseconds

// The state variables
int clock;
uint16_t robot_id;          // Unique robot ID
robot_state_t state;        // State of the robot
double my_pos[3];           // X, Z, Theta of this robot
char target_valid;          // boolean; whether we are supposed to go to the target
double target[99][3];       // x and z coordinates of target position (max 99 targets)
int lmsg, rmsg;             // Communication variables
int indx;                   // Event index to be sent to the supervisor

float buff[99];             // Buffer for physics plugin

double stat_max_velocity;


// Proximity and radio handles
WbDeviceTag emitter_tag, receiver_tag;
static WbDeviceTag ds[NB_SENSORS];  // Handle for the infrared distance sensors
// static WbDeviceTag radio;            // Radio


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* helper functions */

float compute_dist(event_data e1, event_data e2){
    return sqrt((e2.x - e1.x)*(e2.x - e1.x) + (e2.y - e1.y)*(e2.y - e1.y));
}

float compute_delay(event_data e, uint16_t robot_id){
    if (robot_id < 2){              // robot is type A
        if (e.type == A){
            return 3000;    //[ms]
        }
        else{
            return 5000;
        }
    }else{                          // robot is type B
        if (e.type == A){
            return 9000;
        }
        else{
            return 1000;
        }
    }
}

float compute_cost(event_data event_data_list[3], int i, int j, int k){
    float cost = 0.0;
    event_data my_position = {my_pos[0], my_pos[1], 0};
    int indices[3] = {i, j, k}; 

    cost += compute_dist(my_position, event_data_list[indices[0]]) / AVG_SPEED;
    cost += compute_delay(event_data_list[indices[0]], robot_id);

    for (int m = 1; m < 3; m++){
        cost += compute_dist(event_data_list[indices[m-1]], event_data_list[indices[m]]) / AVG_SPEED;
        cost += compute_delay(event_data_list[indices[m]], robot_id);
    }

    return cost;
}

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

// Check if we received a message and extract information
static void receive_updates() 
{
    message_t msg;
    int target_list_length = 0;
    int i;
    int k;

    while (wb_receiver_get_queue_length(receiver_tag) > 0) {

        const message_t *pmsg = wb_receiver_get_data(receiver_tag);
        
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

        //find target list length
        i = 0;
        while(target[i][2] != INVALID){ i++;}
        target_list_length = i;  
        
        if(target_list_length == 0) target_valid = 0; 

        // DEEEEEBUUUUUUUUUUUUUUUG
        //log_message("robot n %d: msg.event_state = %d", robot_id, msg.event_state);
        //log_message("robot n %d, state of the robot: %d", robot_id, state);

        // Event state machine
        if(msg.event_state == MSG_EVENT_GPS_ONLY)
        {
            my_pos[0] = msg.robot_x;
            my_pos[1] = msg.robot_y;
            my_pos[2] = msg.heading;
            continue;
        }
        else if(msg.event_state == MSG_QUIT)
        {
            // Set speed
            wb_motor_set_velocity(left_motor, 0);
            wb_motor_set_velocity(right_motor, 0);
            wb_robot_step(TIME_STEP);
            exit(0);
        }
        else if (msg.event_state == MSG_EVENT_BEING_HANDLED ){    
            state = HANDLING_TASK;
            //log_message("robot_id = %d took the state %d", robot_id, HANDLING_TASK); // pour print dans un fichier txt 
        }
        else if(msg.event_state == MSG_EVENT_DONE)
        {
            state = STAY;
            log_message("BACK TO STAY");

            // If event is done, delete it from array 
            for(i=0; i<=target_list_length; i++)
            {
                if((int)target[i][2] == msg.event_id) 
                { //look for correct id (in case wrong event was done first)
                    for(; i<=target_list_length; i++)
                    { //push list to the left from event index
                        target[i][0] = target[i+1][0];
                        target[i][1] = target[i+1][1];
                        target[i][2] = target[i+1][2];
                    }
                }
            }
            // adjust target list length
            if(target_list_length-1 == 0) target_valid = 0; //used in general state machine 
            target_list_length = target_list_length-1;    
        }
        else if(msg.event_state == MSG_EVENT_WON)
        {
            // insert event at index
            for(i=target_list_length; i>=msg.event_index; i--)
            {
                target[i+1][0] = target[i][0];
                target[i+1][1] = target[i][1];
                target[i+1][2] = target[i][2];
            }
            target[msg.event_index][0] = msg.event_x;
            target[msg.event_index][1] = msg.event_y;
            target[msg.event_index][2] = msg.event_id;
            target_valid = 1; //used in general state machine
            target_list_length = target_list_length+1;
        }
        // check if new bundle is being auctioned
        else if(msg.event_state == MSG_BUNDLE_NEW)
        {
            
            if (target_list_length == 0){     //  CAREFULL MIGHT CAUSE AN ERROR
                event_data event_data_list[3];
                event_data_list[0] = (event_data){msg.a1_x, msg.a1_y, msg.a1_type};
                event_data_list[1] = (event_data){msg.a2_x, msg.a2_y, msg.a2_type};
                event_data_list[2] = (event_data){msg.a3_x, msg.a3_y, msg.a3_type};

                float cost = 0.0;
                float best_cost = -1.0;

                int best_arrangement[3] = {0, 0, 0};

            
                for (int i = 0; i < 3; ++i) {
                    for (int j = 0; j < 3; ++j) {
                        if (j != i) {
                            for (int k = 0; k < 3; ++k) {
                                if (k != i && k != j) {
                                    cost = compute_cost(event_data_list, i, j, k); 
                                    if (best_cost == -1.0 || cost < best_cost){
                                        
                                        best_cost = cost;
                                        best_arrangement[0] = i;
                                        best_arrangement[1] = j;
                                        best_arrangement[2] = k;
                                    }
                                }
                            }
                        }
                    }
                }
              
                const bid_t my_bid = {robot_id, best_cost, {best_arrangement[0], best_arrangement[1], best_arrangement[2]}};
                wb_emitter_set_channel(emitter_tag, robot_id+1);
                wb_emitter_send(emitter_tag, &my_bid, sizeof(bid_t));  

                // Debug print for calculated values
                //log_message("robot_id = %d, distance_to_task = %.2f, time_to_target = %.2f, time_to_handle_task = %.2f, total_time = %.2f", 
                //            robot_id, distance_to_task, time_to_target, time_to_handle_task, total_time);

                // const bid_t my_bid = {robot_id, msg.event_id, total_time, indx};
                // log_message("robot_id = %d bid = %.2f to task %d", 
                //             robot_id, total_time, msg.event_id);
                // log_message("robot_id = %d has the state %d", 
                //             robot_id, state);  

            }
        }
    }

    
    // Communication with physics plugin (channel 0)            
    i = 0; k = 1;
    while((int)target[i][2] != INVALID){i++;}
    target_list_length = i; 
    if(target_list_length > 0)
    {        
        // Line from my position to first target
        wb_emitter_set_channel(emitter_tag,0);         
        buff[0] = BREAK; // draw new line
        buff[1] = my_pos[0]; 
        buff[2] = my_pos[1];
        buff[3] = target[0][0];
        buff[4] = target[0][1];
        // Lines between targets
        for(i=5;i<5*target_list_length-1;i=i+5)
        {
            buff[i] = BREAK;
            buff[i+1] = buff[i-2]; 
            buff[i+2] = buff[i-1];
            buff[i+3] = target[k][0]; 
            buff[i+4] = target[k][1];
            k++;  
        }
        // send, reset channel        
        if(target[0][2] == INVALID){ buff[0] = my_pos[0]; buff[1] = my_pos[1];}
        wb_emitter_send(emitter_tag, &buff, (5*target_list_length)*sizeof(float));
        wb_emitter_set_channel(emitter_tag,robot_id+1);                     
    }
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* RESET and INIT (combined in function reset()) */
void reset(void) 
{
    wb_robot_init();
    int i;
    energy_level = max_energy; 
    //get motor
    left_motor = wb_robot_get_device("left wheel motor");
    right_motor = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);
  
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
    for(i=0;i<99;i++){ 
        target[i][0] = 0;
        target[i][1] = 0;
        target[i][2] = INVALID; 
    }

    // Start in the DEFAULT_STATE
    state = DEFAULT_STATE;

    // read robot id and state from the robot's name
    char* robot_name; 
    robot_name = (char*) wb_robot_get_name();
    int tmp_id;
    if (sscanf(robot_name, "e-puck%d", &tmp_id)) {robot_id = (uint16_t)tmp_id;} 
    else {fprintf(stderr, "ERROR: couldn't parse my id %s \n", robot_name); exit(1);}

    // Am I used in this simulation?
    if (robot_id >= NUM_ROBOTS) {
        fprintf(stderr, "Robot %d is not needed. exiting ...\n", robot_id); 
        wb_robot_cleanup(); 
        exit(0);
    }

    // Link with webots nodes and devices (attention, use robot_id+1 as channels, because
    // channel 0 is reseved for physics plugin)
    emitter_tag = wb_robot_get_device("emitter");
    wb_emitter_set_channel(emitter_tag, robot_id+1);

    receiver_tag = wb_robot_get_device("receiver");
    wb_receiver_enable(receiver_tag, RX_PERIOD); // listen to incoming data every 1000ms
    wb_receiver_set_channel(receiver_tag, robot_id+1);
    
    // Seed random generator
    srand(getpid());

    // Reset stats
    stat_max_velocity = 0.0;
}

void update_state(int _sum_distances)
{

    if (state == GO_TO_GOAL && _sum_distances > STATECHANGE_DIST){
        state = OBSTACLE_AVOID;
    }
    else if(state == OBSTACLE_AVOID && _sum_distances < STATECHANGE_DIST && target_valid){
        state = GO_TO_GOAL;
    }
    else if(state == STAY && target_valid){
        state = GO_TO_GOAL;
    }
}

// Odometry
void update_self_motion(int msl, int msr) {
    double theta = my_pos[2];
  
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
    my_pos[2] -= dtheta;
    
    // Keep orientation within 0, 2pi
    if (my_pos[2] > 2*M_PI) my_pos[2] -= 2.0*M_PI;
    if (my_pos[2] < 0) my_pos[2] += 2.0*M_PI;

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

// Computes wheel speed to go towards a goal
void compute_go_to_goal(int *msl, int *msr) 
{
    // Compute vector to goal
    float a = target[0][0] - my_pos[0];
    float b = target[0][1] - my_pos[1];

    // Compute wanted position from event position and current location
    float x =  a * cosf(my_pos[2]) - b * sinf(my_pos[2]); // x in robot coordinates
    float y =  a * sinf(my_pos[2]) + b * cosf(my_pos[2]); // y in robot coordinates

    // Control coefficients
    float Ku = 0.2;   // Forward control coefficient
    float Kw = 5.0;   // Rotational control coefficient

    // Range and bearing
    float range = sqrtf(x*x + y*y);   // Distance to the wanted position
    float bearing = atan2(y, x);     // Orientation of the wanted position

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


// RUN e-puck
void run(int ms)
{
    for (int i = 0; i<5; i++){
        log_message("robot %d: target %d: x = %f, z = %f \n", robot_id, i, target[i][0], target[i][1]);
    }

    float msl_w, msr_w;
    // Motor speed and sensor variables	
    int msl=0,msr=0;            // motor speed left and right
    int distances[NB_SENSORS];  // array keeping the distance sensor readings
    int sum_distances=0;        // sum of all distance sensor inputs, used as threshold for state change.  	

    // Other variables
    int sensor_nb;

    // Add the weighted sensors values
    for(sensor_nb=0;sensor_nb<NB_SENSORS;sensor_nb++)
    {  
        if (sensor_nb != 3 || sensor_nb != 4){  // don't look at the sensor of the back
            distances[sensor_nb] = wb_distance_sensor_get_value(ds[sensor_nb]);
            sum_distances += distances[sensor_nb];
        }
    }
    //log_message("sum distance = %d for robot %d" , sum_distances, robot_id);
    
    // Get info from supervisor
    receive_updates();
    if (energy_level > 0){
        if (state == GO_TO_GOAL || state == OBSTACLE_AVOID || state == HANDLING_TASK) {
            energy_level -= ms; // Decrement energy by timestep
        }
        // State may change because of obstacles
        update_state(sum_distances);

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
                compute_go_to_goal(&msl, &msr);
                break;

            case OBSTACLE_AVOID:
                compute_avoid_obstacle(&msl, &msr, distances);
                break;

            case RANDOM_WALK:
                msl = 400;
                msr = 400;
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

        // Update clock
        clock += ms;
    } else {
        // Stop movement when out of energy
        wb_motor_set_velocity(left_motor, 0);
        wb_motor_set_velocity(right_motor, 0);
        log_message("robot_id=%d has no energy left!", robot_id);
    }
}

// MAIN
int main(int argc, char **argv) 
{
    reset();
    // RUN THE MAIN ALGORIHM
    while (wb_robot_step(TIME_STEP) != -1) {run(TIME_STEP);}
    wb_robot_cleanup();

    return 0;
}