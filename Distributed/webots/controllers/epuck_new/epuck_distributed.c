  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * file:        e-puck.c
 * author:      
 * description: E-puck file for distributed market-based task allocations (DIS project topic 4)
 *
 * $Revision$	february 2016 by Florian Maushart
 * $Date$
 * $Author$      Last update 2024 by Jacques Benand
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
#define MAX_VELOCITY        0.5 // Maximum robot speed in m/s

#define INVALID          -999
#define BREAK            -999 //for physics plugin

#define NUM_ROBOTS 5 // Change this also in the supervisor!
#define NUM_TASKS 10

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Collective decision parameters */

#define STATECHANGE_DIST 500   // minimum value of all sensor inputs combined to change to obstacle avoidance mode

typedef enum {
    STAY            = 1,
    GO_TO_GOAL      = 2,                    // Initial state aliases
    OBSTACLE_AVOID  = 3,
    RANDOM_WALK     = 4,
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
WbDeviceTag emitter_tag, receiver_tag, sr_emitter_tag, sr_receiver_tag;
static WbDeviceTag ds[NB_SENSORS];  // Handle for the infrared distance sensors
WbDeviceTag gps; // Handle for the gps
#define SR_COM_RANGE = 0.3
// static WbDeviceTag radio;            // Radio

// Structure that contains task information from other robots
typedef struct {
    uint16_t robot_id;
    uint16_t task_id;
    double bid_value; // Time to complete task
} bid_message_t;

// Structure that contains information about a task to be broadcasted to the robots from the supervisor
typedef struct {
    uint16_t task_id;
    double task_x, task_y;
    Event_type task_type; // A or B
} task_message_t;

// Structure that contains information about a task 
typedef struct {
    uint16_t task_id;
    double task_x;
    double task_y;
    bool task_type; // 0 for A and 1 for B
    double best_bid_value; // Lowest bid value known
    uint16_t best_bid_robot; // Robot ID of best bidder
    int is_completed;
} task_t;

// The state variables
int clock;
uint16_t robot_id;          // Unique robot ID
bool robot_type;            // 0 for task A and 1 for task B
robot_state_t state;                 // State of the robot
double my_pos[3];           // X, Z, Theta of this robot
char target_valid;          // boolean; whether we are supposed to go to the target
double target[99][3];       // x and z coordinates of target position (max 99 targets)
task_t tasks[NUM_TASKS];    // Array containing all the tasks present on the field
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

// Compute the bid for the appropriate task
double compute_bid(task_t task) {
    double distance = dist(task.task_x - my_pos[0], task.task_y - my_pos[1]);
    double completion_time = 0;
    if (task.task_type == robot_type) {
        if (robot_type == 0) completion_time = 3;
        else completion_time = 1;
    } else {
        if (robot_type == 0) completion_time = 5;
        else completion_time = 9;
    }
    return distance*MAX_VELOCITY + completion_time
}

// Send the bid to other robots in communication range
void broadcast_bid(task_t task) {
    bid_message_t bid = {robot_id, task.task_id, compute_bid(task)};
    wb_emitter_send(sr_emitter_tag, &bid, sizeof(bid_message_t));
}

 // Claim a task 
void claim_task(task_t *task) {
    task->best_bid_robot = robot_id;
    printf("Robot %d claims task %d/n", robot_id, task->task_id);
}

// Complete a task
void complete_task(task_t *task) {
    task->is_completed = 1;
    printf("Robot %d completed task %d/n", robot_id, task->task_id);
}

// Update the position depending on the GNSS measurements
void update_position() {
    const double *gps_values = wb_gps_get_values(gps);
    my_pos[0] = gps_values[0];
    my_pos[1] = gps_values[1];
}

// Check if we received a message and extract information
static void receive_updates() 
{
    message_t msg;
    message_t sr_msg;
    int target_list_length = 0;
    int i;
    int k;

    while (wb_receiver_get_queue_length(sr_receiver_tag) > 0) {
        const message_t *sr_pmsg = wb_receiver_get_data(sr_receiver_tag);
        // save a copy, cause wb_receiver_next_packet invalidates the pointer
        memcpy(&sr_msg, sr_pmsg, sizeof(message_t));
        wb_receiver_next_packet(sr_receiver_tag);

        // double check this message is for me
        // communication should be on specific channel per robot
        // channel = robot_id + 1, channel 0 reserved for physics plguin
        if(sr_msg.robot_id != robot_id) {
            fprintf(stderr, "Invalid message: robot_id %d "  "doesn't match receiver %d\n", sr_msg.robot_id, robot_id);
            //return;
            exit(1);
        }

        // Update the tasks info depending on the message
        for (int i=0; i<NUM_TASKS; i++) {
            if (tasks[i].task_id == sr_msg->task_id and !tasks[i].is_completed) {
                if (msg->bid_value < tasks[i].best_bid_value) {
                    tasks[i].best_bid_value = sr_msg->bid_value;
                    tasks[i].best_bid_robot = sr_msg->robot_id;
                }
            }
        }
    }

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
        else if(msg.event_state == MSG_EVENT_DONE)
        {
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
        // check if new event is being auctioned
        else if(msg.event_state == MSG_EVENT_NEW)
        {                
                //*********         INSERT YOUR TACTIC BELOW         *********//
                //                                                            //
                // Determine your bid "d" and the index "indx" at             //
                // which you want to insert the event into the target list.   //
                // Available variables:                                       //
                // my_pos[0], my_pos[1]: Current x and y position of e-puck   //
                // msg.event_x, msg.event_z: Event x and y position           //
                // target[n][0], target[n][1]: x and y pos of target n in your//
                //                             target list                    //
                // target_list_length: current length of your target list     //
                //                                                            //
                // You can use dist(ax, ay, bx, by) to determine the distance //
                // between points a and b.                                    //
                ////////////////////////////////////////////////////////////////

            ///*** SIMPLE TACTIC ***///
            //indx = target_list_length;
            //double d = dist(my_pos[0], my_pos[1], msg.event_x, msg.event_y);
            ///*** END SIMPLE TACTIC ***///
                

            ///*** BETTER TACTIC ***///
            // Place your code here for I17 
            
            indx = 0;
            double d = 0;
            if(target_list_length > 0){
			    // TODO: put your strategy here
               for(i=0; i<target_list_length; i++){
                 double d_new = dist(my_pos[0], my_pos[1], target[i][0], target[i][1]);
                 if(d_new<d || d==0){
                   indx = i;
                   d = d_new;
                   }
                   }
		    }else{
	d = 5000;             
		    }
		    
            ///*** END BETTER TACTIC ***///
                
                
            ///*** BEST TACTIC ***/// 
		    // Place your code here for I20
            //indx = 0;
            //double d = dist(my_pos[0], my_pos[1], msg.event_x, msg.event_z);             
            //if(target_list_length > 0)
            //{  
                // for all the tasks inside the task list (i.e. target[i] where i goes up to target_list_length)  check if putting the current 
                // event (located at (msg.event_x, msg.event_z)) in between two task results in  a smaller distance, and modify the d accordingly.
            //}

            ///*** END BEST TACTIC ***///
                
            // Send my bid to the supervisor
            const bid_t my_bid = {robot_id, target[indx][2], d, indx};
            wb_emitter_set_channel(emitter_tag, robot_id+1);
            wb_emitter_send(emitter_tag, &my_bid, sizeof(bid_t));            
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

    //get motors
    left_motor = wb_robot_get_device("left wheel motor");
    right_motor = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);

    // Initialize the emitters and receivers for communication with the supervisor
    emitter_tag = wb_robot_get_device("emitter");
    receiver_tag = wb_robot_get_device("receiver");
    wb_receiver_enable(receiver_tag, RX_PERIOD); // listen to incoming data every 1000ms
    wb_receiver_set_channel(receiver_tag, robot_id+1);

    // Initialize the emitters and receiver for short communication (inter robot communication)
    sr_emitter_tag = wb_robot_get_device("emitter");
    wb_emitter_set_channel(sr_emitter_tag, robot_id+NUM_ROBOTS+1);
    wb_emitter_set_range(sr_emitter_tag, SR_COM_RANGE);
    sr_receiver_tag = wb_robot_get_device("receiver");
    wb_receiver_enable(sr_receiver_tag, TIME_STEP);
    wb_receiver_set_channel(sr_receiver_tag, robot_id+NUM_ROBOTS+1);

    // Enable GNSS localization
    gps = wb_robot_get_device("gps");
    wb_gps_enable(gps, TIME_STEP);
  
    char s[4] = "ps0";
    for(i=0; i<NB_SENSORS;i++) 
    {
        // the device name is specified in the world file
        ds[i] = wb_robot_get_device(s);      
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
    
    // Seed random generator
    srand(getpid());

    // Reset stats
    stat_max_velocity = 0.0;
}


void update_state(int _sum_distances)
{
    if (_sum_distances > STATECHANGE_DIST && state == GO_TO_GOAL)
    {
        state = OBSTACLE_AVOID;
    }
    else if (target_valid)
    {
        state = GO_TO_GOAL;
    }
    else
    {
        state = DEFAULT_STATE;
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

// Compute wheel to go towards a task
void compute_go_to_task(task_t task, int *msl, int *msr)
{
    // Compute vector to goal
    float a = task.task_x - my_pos[0];
    float b = task.task_y - my_pos[1];
    // Compute wanted position from task positioin and current location
    float x = a*cosf(my_pos[2]) - b*sinf(my_pos[2]);
    float y = a*sinf(my_pos[2]) + b*cosf(my_pos[2]);

    float Ku = MAX_VELOCITY;     // Forward control gain
    float Kw = 10.0;    // Rotation control gain
    float range = 1;
    float bearing = atan2(y, x);    // Orientation of the wanted position

    // Compute forward control
    float u = Ku*range*cosf(bearing);
    //Compute rotation control
    float w = Kw*range*sinf(bearing);

    // Convert to wheel speeds
    *msl = 50*(u - AXLE_LENGTH*w/2.0) / WHEEL_RADIUS;
    *msr = 50*(u + AXLE_LENGTH*w/2.0) / WHEEL_RADIUS;
    limit(msl, MAX_SPEED);
    limit(msr, MAX_SPEED);
}

void handle_communication()
{
    N = wb_receiver_get_queue_length(sr_receiver_tag);
    for (int i=0; i<N; i++) {
        sr_pkt = (double*)wb_receiver_get_data(sr_receiver);
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

    // Handle communication with other robots
    handle_communication();

    // State may change because of obstacles
    update_state(sum_distances);

    // Set wheel speeds depending on state
    switch (state) {
        case STAY:
            msl = 0;
            msr = 0;
            break;

        case GO_TO_GOAL:
            compute_go_to_task(task, &msl, &msr); //CHANGE?????????????????????????????????????????????????????????????????????????????????????????????????????????
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