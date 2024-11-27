/*****************************************************************************/
/* File:         guided_con.c                                                */
/* Version:      1.0                                                         */
/* Date:         30-Nov-14                                                   */
/* Description:  This is an incomplete robot controller for maximizing       */
/*               information gain about a light field.                       */
/* Author:       adrian.arfire@epfl.ch                                       */
/*****************************************************************************/
#include <webots/robot.h>
#include <webots/light_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/distance_sensor.h>
#include <webots/gps.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <webots/motor.h>
#include <time.h>       /* time */


#define TIME_STEP 64
#define COM_RANGE 1.5   // Transmission range in meters
#define SHORT_COM_RANGE 0.7   // Transmission range in meters
#define F_M 1000/TIME_STEP // Fm - maximum message transmission rate
#define RANGE 1024/2
#define BASELINE 500
#define V_MAX 0.12874          // v_max - maximum robot velocity (m/s)

#define AXLE_LENGTH         0.052   // Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS     0.00628 // Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS        0.0205  // Wheel radius (meters)
#define DELTA_T             TIME_STEP/1000   // Timestep (seconds)
#define MAX_SPEED_WEB      6.28    // Maximum speed webots
#define UPDATE_RATE 10
#define RS_SCALE (1.0 / (1.0 + RAND_MAX))
#define N_DIST_SENS 8

int pkt_count = 0; // counts number of packets sent by this robot
double buffer[3], sensor_value, prev_ls, grad, ds = 0;
double persBest[2]; // location of personal best
double persBest_v = 0; // value of personal best
double ngbrBest[2]; // location of neighborhood best
double ngbrBest_v = 0; // value of neighborhood best
double weights[4] = {0.1,0.4,0.45,0.05};  // weight of each velocity term
double speed[2];    // wheel speeds
double prev_speed[2];    // wheel speeds
int robot_id;                               // Unique robot ID


static WbDeviceTag emitter, receiver;       // Radio
static WbDeviceTag sr_emitter, sr_receiver; // Short range radio
WbDeviceTag left_motor; //handler for left wheel of the robot
WbDeviceTag right_motor; //handler for the right wheel of the robot
WbDeviceTag turret_sensor;
WbDeviceTag distance_sensor[N_DIST_SENS];
WbDeviceTag gps;

double braitenberg_coefficients[N_DIST_SENS][2] =
    { {150, -35}, {100, -15}, {80, -10}, {-10, -10},
    {-10, -10}, {-10, 80}, {-30, 100}, {-20, 150} };

typedef enum {
    STAY            = 1,
    GO_TO_GOAL      = 2,                    // Initial state aliases
    OBSTACLE_AVOID  = 3,
} robot_state_t;

#define STATECHANGE_DIST 500 // minimum value of all sensor inputs combined to change to obstacle avoidance state
#define DEFAULT_STATE (STAY)

#define NB_SENSORS           8
#define BIAS_SPEED           400

// Weights for the Braitenberg algorithm
// NOTE: Weights from reynolds2.h
int Interconn[16] = {17,29,34,10,8,-38,-56,-76,-72,-58,-36,8,10,36,28,18};

// The state variables
int clock;
//uint16_t robot_id;          // Unique robot ID
robot_state_t state;        // State of the robot
int robot_specialization;   // 0 for Task A and 1 for Task B
double my_pos[3];           // X, Z, Theta of this robot
char target_valid;          // boolean; whether we are supposed to go to the target
double target[99][3];       // x and z coordinates of target position (max 99 targets)
int lmsg, rmsg;             // Communication variables
int indx;                   // Event index to be sent to the supervisor
double energy = 120.0;      // Robot's energy level

float buff[99];             // Buffer for physics plugin

double stat_max_velocity;

// Proximity and radio handles
WbDeviceTag emitter_tag, receiver_tag;
static WbDeviceTag ds[NB_SENSORS];  // Handle for the infrared distance sensors
// static WbDeviceTag radio;            // Radio
////// --------------------- FUNCTIONS -----------------------------------------//////////

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

double calculate_completion_time(int task_type) {
    if (robot_specialization == task_type) {
        return (task_type == 0) ? 3 : 1; // Specialized: Task A=3s, Task B=1s
    } else {
        return (task_type == 0) ? 9 : 5; // Non-specialized: Task A=9s, Task B=5s
    }
}

double calculate_bid(double distance, int task_type) {
    double travel_time = distance / 0.5; // Speed = 0.5 m/s
    double completion_time = calculate_completion_time(task_type);
    printf("Robot ID: %d, Distance: %.2f, Task Type: %d, Bid: %.2f\n", robot_id, distance, task_type, calculate_bid(distance, task_type));

    return travel_time + completion_time;
}

/*
 * Reset the robot's devices and get its ID
 *
 */

static void reset(){
  /* initialize Webots */
  wb_robot_init();
  
  // read robot id from the robot's name
  char* robot_name;
  robot_name=(char*) wb_robot_get_name(); 
  sscanf(robot_name,"e-puck%d",&robot_id);

  // Initialize emitter and receiver
  emitter = wb_robot_get_device("emitter");
  wb_emitter_set_range(emitter,COM_RANGE);
  receiver = wb_robot_get_device("receiver");
  wb_receiver_set_channel(receiver,0);
  wb_receiver_enable(receiver,TIME_STEP);

  //Initialize motors
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);

  /* enable absolute positioningwith GPS*/
  gps = wb_robot_get_device("gps");
  wb_gps_enable(gps,TIME_STEP);

  // Get the position of the robot depending on the GPS measurement
  my_pos = wb_gps_get_values(gps);

  // Compute the bid of the robot on each of the tasks
  for (i=0; i<)
  
  // Enable turret sensor
  turret_sensor = wb_robot_get_device("ts");
  wb_light_sensor_enable(turret_sensor,TIME_STEP);  
  wb_robot_step(TIME_STEP);

    for (int i = 0; i < N_DIST_SENS; i++) {
    char device_name[4];

    /* get distance sensors */
    sprintf(device_name, "ps%d", i);
    distance_sensor[i] = wb_robot_get_device(device_name);
    wb_distance_sensor_enable(distance_sensor[i],TIME_STEP*4);
  }

}


void send_data(double tstamp) {
  buffer[0] = tstamp;
  buffer[1] = robot_id;
  buffer[2] = sensor_value;
  wb_emitter_send(emitter,buffer,3*sizeof(double));
  pkt_count++;
}

bool obstacleDetected(double *ps) {
  if ((ps[0]+ps[1]+ps[2]+ps[5]+ps[6]+ps[7]) > 400) return true;
  else return false;
}

// Odometry
void update_self_motion(int msl, int msr) {
  
    // Compute deltas of the robot
    double dr = (double)msr * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
    double dl = (double)msl * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
    double du = (dr + dl)/2.0;
    ds = du;
}


double computeHeading(const double *pos, double *pos_prev) {
  double x, y;
  x = pos[0] - pos_prev[0];
  y = pos[1] - pos_prev[1];
 
  if ((x == 0) && (y == 0)) return 0; // actually not defined...
  else return atan2(y,x);
}


void compute_wheel_speeds(double *sl, double *sr, const double *curr_pos, double curr_heading, double *target) {
  // Define constants
  double Ku = 2.0;
  double Kw = 20.0;

  // Compute the range and bearing to the wanted position
  double dx = target[0] - curr_pos[0];//goal_range * cosf(- M_PI + goal_bearing);
  double dy = target[1] - curr_pos[1];//goal_range * sinf(- M_PI + goal_bearing);
  double range = sqrtf(dx*dx + dy*dy); // This is the wanted position (range)
    double bearing;
  if ((dy == 0) && (dx == 0)) bearing = 0;
  else bearing = atan2(dy, dx) - curr_heading;    // This is the wanted position (bearing)

  
  // Compute forward control
  double u = Ku * range * cosf(bearing);
  if (u < 0) u = 0; // only move forwards
  // Compute rotional control
  double w = Kw * range * sinf(bearing);

  // Convert to wheel speeds!
  *sl = (u - AXLE_LENGTH*w/2.0) / (SPEED_UNIT_RADS * WHEEL_RADIUS);
  *sr = (u + AXLE_LENGTH*w/2.0) / (SPEED_UNIT_RADS * WHEEL_RADIUS);
}



/*
 * Keep given int number within interval {-limit, limit}
 */
void limitf(float *number, int limit) {

	if (*number > limit)
		*number = (float)limit;
	if (*number < -limit)
		*number = (float)-limit;
}

void update_state(int _sum_distances) {
    printf("Robot ID: %d, Current State: %d, Target Valid: %d\n", robot_id, state, target_valid);

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


///////////// ------------------ MAIN ---------------------------///////////////

int main(int argc, char *argv[]) {
  /* define variables */

  int i, j, N, count = 0;
  double perf_data[3], buffPB[3];
  double ps_value[N_DIST_SENS];
  double clock = 0;
  const double *pos, *sr_pkt;
  double pos_prev[2];
  double heading; // heading computed based on two consecutive position measurements
  float msl_w, msr_w;
  
  reset();

  //Initialize light sensor reading and position
  prev_ls = wb_light_sensor_get_value(turret_sensor);
  pos = wb_gps_get_values(gps);
  
  //Initialize previous position
  pos_prev[0] = pos[0];
  pos_prev[1] = pos[1];
  

  /* main loop */
  while (wb_robot_step(TIME_STEP) != -1) {
    count++;
    clock += (double)TIME_STEP/1000;
    
    /* get proximity sensors values */
    for (i = 0; i < N_DIST_SENS; i++) {
      ps_value[i] = wb_distance_sensor_get_value(distance_sensor[i]);
    }
    
    /* get current position */
    pos = wb_gps_get_values(gps);
    /* get current heading */
    heading = computeHeading(pos,pos_prev);
    
    printf("Rob %d position: %.3f %.3f heading: %.3f\n",robot_id,pos[0],pos[1],heading);

    /* get light sensor value */
    sensor_value = wb_light_sensor_get_value(turret_sensor);
    
    /* compute current gradient */
    grad = sensor_value - prev_ls;
    if (fabs(grad) > persBest_v) {
      persBest_v = fabs(grad);
      persBest[0] = pos[0];
      persBest[1] = pos[1];
    }
    printf("Rob %d curr gradient: %.3f, pers_best: %.3f (%.3f,%.3f) \n",robot_id,grad,persBest_v,persBest[0],persBest[1]);

    /* handle communication */
    if(wb_receiver_get_queue_length(receiver) > 0) {
      wb_emitter_set_channel(emitter,3);
      perf_data[0] = (double)robot_id;
      perf_data[1] = (double)(pkt_count-1);
      perf_data[2] = ds;
      wb_emitter_send(emitter,perf_data,3*sizeof(double));


      wb_motor_set_velocity(left_motor, 0);
      wb_motor_set_velocity(right_motor, 0);
      break; // stop node
    }
    else {
      send_data(clock); // send measurement data
      
      /* handle all incoming short range messages */
      ngbrBest_v = 0;
      N = wb_receiver_get_queue_length(sr_receiver);
      for (i = 0; i < N; i++) {
        sr_pkt = (double*)wb_receiver_get_data(sr_receiver);


        if (sr_pkt[2] > ngbrBest_v) {
          ngbrBest_v = sr_pkt[2];
          ngbrBest[0] = sr_pkt[0];
          ngbrBest[1] = sr_pkt[1];
        }
        
        wb_receiver_next_packet(sr_receiver);
      }
      
      /* compare received best with own best */
      if (persBest_v > ngbrBest_v) {
        ngbrBest_v = persBest_v;
        ngbrBest[0] = persBest[0];
        ngbrBest[1] = persBest[1];
      }
      
      /* send personal best over short range radio */
      buffPB[0] = persBest[0];
      buffPB[1] = persBest[1];
      buffPB[2] = persBest_v;
      wb_emitter_send(sr_emitter,buffPB,3*sizeof(double));
      
      printf("Rob %d ngbr best: %.3f (%.3f,%.3f) \n",robot_id,ngbrBest_v,ngbrBest[0],ngbrBest[1]);
    }
    
    if (!obstacleDetected(ps_value)) {
      if (count%UPDATE_RATE == 1) {
        /* TODO: find wheel speeds by compute_wheel_speeds() using:
            - pos,
            - heading,
            - prev_speed,
            - persBest,
            - ngbrBest,
            
            Then add some randomness to the direction of the robot by compute_wheel_speeds() using:
            - pos,
            - heading,
            - rand()
            
            Then obtain the final speeds (speed[0] and speed[1]) by adding the above obtained wheel speeds, using the 'weights' (already defined)
            
        //speed[0] = 
        //speed[1] = */
        double sl_pBest,sr_pBest,sl_nBest,sr_nBest,sl_rnd,sr_rnd;
        compute_wheel_speeds(&sl_pBest,&sr_pBest,pos,heading,persBest);
        compute_wheel_speeds(&sl_nBest,&sr_nBest,pos,heading,ngbrBest);
        
        double rnd[2];
        srand(time(NULL));
        rnd[0] = 2*rand()*RS_SCALE - 1;
        rnd[1] = 2*rand()*RS_SCALE - 1;
        compute_wheel_speeds(&sl_rnd,&sr_rnd,pos,heading,rnd);
        
        speed[0] = weights[0]*prev_speed[0] + weights[1]*sl_pBest + weights[2]*sl_nBest + weights[3]*sr_rnd;
        speed[1] = weights[0]*prev_speed[1] + weights[1]*sr_pBest + weights[2]*sr_nBest + weights[3]*sl_rnd;  
        
      }
    }
    else {
      /* compute speed values*/
      for (i = 0; i < 2; i++) {
        speed[i] = 0.0;
        for (j = 0; j < N_DIST_SENS; j++) {
          speed[i] += braitenberg_coefficients[j][i] * (1.0 - (ps_value[j] / RANGE));
        }
      }
      speed[0] *= 2;
      speed[1] *= 2;
    }
    
    // Set speed
    msl_w = speed[0]*MAX_SPEED_WEB/1000;
    msr_w = speed[1]*MAX_SPEED_WEB/1000;
	  limitf(&msl_w,MAX_SPEED_WEB);
    limitf(&msr_w,MAX_SPEED_WEB);

    wb_motor_set_velocity(left_motor, msl_w);
    wb_motor_set_velocity(right_motor, msr_w);
    
    /* compute odometry */
    update_self_motion(speed[0],speed[1]);
    
    //printf("speed: %f %f\n",speed[0],speed[1]);
    
    // Keep previous sensor value, postion and speed
    prev_ls = sensor_value;
    pos_prev[0] = pos[0];
    pos_prev[1] = pos[1];  
    prev_speed[0] = speed[0];
    prev_speed[1] = speed[1];
  }
  wb_robot_cleanup();

  return 0;
}
