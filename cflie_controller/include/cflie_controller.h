#include <ros/ros.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>

#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

#include <pc_asctec_sim/pc_goal_cmd.h>
#include <pc_asctec_sim/pc_feedback.h>
#include <pc_asctec_sim/pc_state.h>
#include <cflie_controller/tunerConfig.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <dynamic_reconfigure/server.h>

#include <string.h>
#include <math.h>

#define CONTROL_RATE 20.0

#define INTEGRAL_LIMIT 150.0
#define G_TH 29500.0

#define THRUST_SCALE 1000
#define THRUST_MAX 60000.0
#define THRUST_MIN 10000.0

#define ROLL_MAX 30.0
#define ROLL_MIN -ROLL_MAX

#define PITCH_MAX 30.0
#define PITCH_MIN -PITCH_MAX

#define YAW_MAX 200.0
#define YAW_MIN -YAW_MAX

#define BATTERY_FULL 4.2
#define BATTERY_MID 3.7
#define BATTERY_EMPTY 3.2

using namespace std;

/* ---------------- Prototype Definitions ---------------- */

void init(struct POS_DATA * pos_ptr, 
          struct PID_DATA * ctl_ptr);
void update_controller(struct PID_DATA * controller_ptr, 
                       struct POS_DATA * position_ptr);
void update_real_cmd(struct PID_DATA * ctl_ptr, struct POS_DATA * pos_ptr);
void goal_callback(const pc_asctec_sim::pc_goal_cmd::ConstPtr& msg);
bool goal_arrived(struct PID_DATA * pid_ptr, struct POS_DATA * pos_ptr);
float limit(float input, float ceiling);

/* --------------- Data Structure Definitions ------------ */

typedef struct PID_DATA
{
  float error_x;
  float error_x_vel;
  float error_x_acc;
  float integral_x;

  float error_y;
  float error_y_vel;
  float error_y_acc;
  float integral_y;

  float error_z;
  float error_z_vel;
  float error_z_acc;
  float integral_z;

  float error_yaw;
  float error_yaw_vel;
  float error_yaw_acc;
  float integral_yaw;

} pid_data;

typedef struct POS_DATA
{
  float pos_x;
  float pos_y;
  float pos_z;
  float pos_yaw;
  float relative_yaw;

  float pos_x_past;
  float pos_y_past;
  float pos_z_past;
  float pos_yaw_past;

  float vel_x;
  float vel_y;
  float vel_z;
  float vel_yaw;

  float vel_x_past;
  float vel_y_past;
  float vel_z_past;
  float vel_yaw_past;

  float acc_x;
  float acc_y;
  float acc_z;
  float acc_yaw;

  float goal_x;
  float goal_y;
  float goal_z;
  float goal_yaw;

  float goal_vel_x;
  float goal_vel_y;
  float goal_vel_z;
  float goal_vel_yaw;

  float goal_acc_x;
  float goal_acc_y;
  float goal_acc_z;
  float goal_acc_yaw;

  float goal_range;
  int wait_time;
  int wait_start;
  bool waiting;
  bool goal_arrival;
  string goal_id;

} pos_data;
