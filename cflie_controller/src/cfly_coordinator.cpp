#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <pc_asctec_sim/pc_traj_cmd.h>
#include <pc_asctec_sim/pc_state.h>
#include <math.h>
#include <string.h>
#include <iostream>
#include <stdlib.h>
#include <algorithm>

#define freq 10.0

#define OFFSET 1.2
#define TASKS 8
#define START_TASKS 5
#define kp 4.0
#define kt 12.0
#define kd 6.0
#define kl 4.0
#define kb 70.0 //80 too high
#define kdb 70.0

using namespace std;

ros::Publisher traj_pub, viz_pub, stop_pub, bat_pub, task_pub, border_pub;
ros::Subscriber halt_sub, traj_sub, qb_sub, tb_sub, joy_sub;
ros::Timer timer_event;
ros::Timer life_event;

double dt = 0.0; 
ros::Time past, now;

float pre_x, pre_y, pre_z, pre_xv, pre_yv, pre_zv;
float tLand = 1;

struct sM {
	bool ishalted;
	bool traj;
	int state;
	int taskIs;
} machine;

struct task {
	float time;
	float lifeTime;
	float x;
	float y;
	float z;
	float priority;
	float cost;
	bool isDone;
} task_data;

struct robot_data {
	string frame;
	string world;
	float b_points[16];
	int b_count;
	float battery;
	float v2d;
	float b_min;
	float b_max;

	float x;
	float y;
	float z;
	float xp;
	float yp;
	float zp;
	float yaw;
	float vx;
	float vy;
	float vz;
	
	float max_v;
} robot;

robot_data quad;
robot_data t_bot;
robot_data * quad_ptr = &quad;
robot_data * tur_ptr = &t_bot;

task tasks[TASKS];
sM st_mach;

/* ----- Callbacks ----- */
void joy_callback(const sensor_msgs::Joy::ConstPtr& msg)
{
	if(msg->buttons[0]) {
		ROS_INFO("External start joy-command called!");
		st_mach.ishalted = false;

	}else if(msg->buttons[1]) {
		ROS_INFO("External halt joy-command called!");
		st_mach.state = 2;
		st_mach.ishalted = true;

	}
}

void halt_callback(const std_msgs::Bool::ConstPtr& msg)
{
	if(msg->data) {
		ROS_INFO("External halt command called!");
		st_mach.state = 2;
		st_mach.ishalted = true;
	}else {
		ROS_INFO("External start command called!");
		st_mach.ishalted = false;		
	}
}

void traj_call(const std_msgs::Empty::ConstPtr& msg)
{
	st_mach.traj = true;
	if(st_mach.taskIs != -1) {
		tasks[st_mach.taskIs].isDone = true;
	}
}

void qb_call(const std_msgs::Float32::ConstPtr& msg)
{
	quad.b_points[quad.b_count] = msg->data;
	quad.b_count++;
	quad.b_count &= 15;
	quad.battery = 0.0;
	for(int i = 0; i < 16; i++) {
		quad.battery += quad.b_points[quad.b_count];
	}
	quad.battery = quad.battery / 16;
}

void tb_call(const std_msgs::Float32::ConstPtr& msg)
{
	t_bot.b_points[t_bot.b_count] = msg->data;
	t_bot.b_count++;
	t_bot.b_count &= 15;
	t_bot.battery = 0.0;
	for(int i = 0; i < 16; i++) {
		t_bot.battery += t_bot.b_points[t_bot.b_count];
	}
	t_bot.battery = t_bot.battery / 16;
}

void timerCallback(const ros::TimerEvent&) 
{
	int newTask = -1;
	for(int i = 2; i < TASKS; i++) {
		if(tasks[i].isDone) {
			newTask = i;
			break;
		}
	}
	if(newTask != -1) {
		tasks[newTask].x = float(rand() % 240) / 100 - 1.2;
		tasks[newTask].y = float(rand() % 300) / 100 - 1.8;
		tasks[newTask].z = 1.0;
		tasks[newTask].time = rand() % 5 + 1;
		tasks[newTask].lifeTime = 0;
		tasks[newTask].priority = rand() % 10 + 1;
		tasks[newTask].isDone = false;
		ROS_INFO("New Task added at x: %f, y: %f", tasks[newTask].x, tasks[newTask].y);

	}else {
		ROS_INFO("Task Queue Full!!");

	}
}

void lifetimerCallback(const ros::TimerEvent&) 
{
	for(int i = 2; i < TASKS; i++) {
		if(!tasks[i].isDone) {
			tasks[i].lifeTime++;
		}
	}
}

/* ----- Inits ----- */
void init(string quad_str, string turtle_str, string world_str) 
{
	quad.frame = quad_str;
	quad.world = world_str;
	quad.x = quad.y = quad.z = 0.0;
	quad.max_v = 0.3;

	quad.v2d = 9.5;
	quad.battery = 4.0;
	quad.b_min = 3.15;
	quad.b_max = 4.1;

	for(int i = 0; i < 16; i++) {
		quad.b_points[i] = 4.0;
	}
	quad.b_count = 0;
	
	t_bot.frame = turtle_str;
	t_bot.world = world_str;
	t_bot.x = t_bot.y = t_bot.z = 0.0;
	t_bot.xp = t_bot.yp = t_bot.zp = 0.0;

	t_bot.battery = 4.0;
	for(int i = 0; i < 16; i++) {
		t_bot.b_points[i] = 4.0;
	}
	t_bot.b_count = 0;

	st_mach.ishalted = false;
	st_mach.traj = false;
	st_mach.state = 0;
}

void init_tasks(void)
{
	for(int i = 2; i < START_TASKS; i++) {
		tasks[i].x = float(rand() % 240) / 100 - 1.2;
		tasks[i].y = float(rand() % 300) / 100 - 1.8;
		tasks[i].z = 1.0;
		tasks[i].time = rand() % 5 + 1;
		tasks[i].lifeTime = 0;
		tasks[i].priority = rand() % 10 + 1;
		tasks[i].isDone = false;
	}

	for(int i = START_TASKS; i < TASKS; i++) {
		tasks[i].isDone = true;
	}
}

/* ----- Viz Functions ----- */
void show_battery(float radius)
{
	visualization_msgs::Marker ring;
	geometry_msgs::Point vis_ring;

	ring.header.frame_id = quad.world;
	ring.header.stamp = ros::Time::now();
	ring.id = 2;
	ring.action = visualization_msgs::Marker::ADD;
	ring.type = visualization_msgs::Marker::LINE_LIST;
	ring.color.a = 1.0;
	ring.color.g = 1.0;				

	ring.scale.x = 0.05;
	ring.scale.y = 0.05;

	float i = 0.0;
	while(i < 2*M_PI) {
		vis_ring.x = cos(i) * radius;
		vis_ring.y = sin(i) * radius;
		ring.points.push_back(vis_ring);
		i += 0.05;

		vis_ring.x = cos(i) * radius;
		vis_ring.y = sin(i) * radius;
		ring.points.push_back(vis_ring);
	}

	bat_pub.publish(ring);
}

void show_border(void)
{
	visualization_msgs::Marker border;
	geometry_msgs::Point corner;

	border.header.frame_id = quad.world;
	border.header.stamp = ros::Time::now();
	border.id = 2;
	border.action = visualization_msgs::Marker::ADD;
	border.type = visualization_msgs::Marker::LINE_LIST;

	border.color.a = 1.0;
	border.color.r = 1.0;				
	border.scale.x = 0.05;
	border.scale.y = 1.0;
	border.scale.z = 1.0;

	corner.x = -1.2;
	corner.y = -1.8;	
	corner.z = 0.75;
	border.points.push_back(corner);

	corner.x = -1.2;
	corner.y = 1.8;	
	corner.z = 0.75;
	border.points.push_back(corner);

	corner.x = -1.2;
	corner.y = 1.8;	
	corner.z = 0.75;
	border.points.push_back(corner);

	corner.x = 1.2;
	corner.y = 1.8;	
	corner.z = 0.75;
	border.points.push_back(corner);

	corner.x = 1.2;
	corner.y = 1.8;	
	corner.z = 0.75;
	border.points.push_back(corner);

	corner.x = 1.2;
	corner.y = -1.8;	
	corner.z = 0.75;
	border.points.push_back(corner);

	corner.x = 1.2;
	corner.y = -1.8;	
	corner.z = 0.75;
	border.points.push_back(corner);

	corner.x = -1.2;
	corner.y = -1.8;	
	corner.z = 0.75;
	border.points.push_back(corner);

	border_pub.publish(border);
}

void show_tasks(void)
{
	visualization_msgs::Marker task_list;
	geometry_msgs::Point task;

	task_list.header.frame_id = quad.world;
	task_list.header.stamp = ros::Time::now();
	task_list.id = 0;
	task_list.action = visualization_msgs::Marker::ADD;
	task_list.type = visualization_msgs::Marker::POINTS;
	task_list.color.a = 1.0;
	task_list.color.b = 1.0;				

	task_list.scale.x = 0.05;
	task_list.scale.y = 0.05;

	for(int i = 2; i < TASKS; i++) {
		if(!tasks[i].isDone) {
			task.x = tasks[i].x;
			task.y = tasks[i].y;
			task.z = tasks[i].z;
			task_list.points.push_back(task);
		}
	}

	task_pub.publish(task_list);
}

/* ----- Pub Functions ----- */

void send_trajectory(float time, float wait, float x, float y, float z)
{
	pc_asctec_sim::pc_traj_cmd cmd_point;
	cmd_point.x_end[0] = x;
	cmd_point.y_end[0] = y;
	cmd_point.z_end[0] = z;
	
	cmd_point.wait_time[0] = wait;
	cmd_point.duration[0] = time;
	cmd_point.points = 1;
	traj_pub.publish(cmd_point);
}

/* ----- General Functions ----- */
float getTime(float x, float y, float z) 
{
	float d_target = sqrt(pow(x - quad.x,2) + pow(y - quad.y,2) + pow(z - quad.z,2));
	
	return d_target/quad.max_v;
}

int setCosts(void)
{
/*	Cost_task = kd*(t_travel) + kt*(t_task) - kp*(priority) - kl*(lifeTime)
	Cost_batt = min(kd*(t_center + t_land) - kb*(battery), kd*(t_turtle + t_land) - kb*(battery))
*/
	int low_cost = -1;
	float lowest_cost = 10000;
	float t_center = (sqrt(pow(quad.x,2) + pow(quad.y,2) + pow(quad.z - 0.0,2)))/quad.max_v;
	float t_turtle = (sqrt(pow(quad.x - t_bot.x,2) + pow(quad.y - t_bot.y,2) + pow(quad.z - t_bot.z + 0.05,2)))/quad.max_v;

	tasks[0].cost = kdb*(t_turtle + tLand) - kb*(quad.b_max - quad.battery);
	tasks[1].cost = kdb*(t_center + tLand) - kb*(quad.b_max - quad.battery);
	
	if(tasks[0].cost < lowest_cost) {
		low_cost = 0;
		lowest_cost = tasks[0].cost;
	}

	/*if(tasks[1].cost < lowest_cost) {
		low_cost = 1;
		lowest_cost = tasks[1].cost;
	}*/

	for(int i = 2; i < TASKS; i++) {
		if(!tasks[i].isDone) {
			float t_travel = (sqrt(pow(quad.x - tasks[i].x,2) + pow(quad.y - tasks[i].y,2) + pow(quad.z - tasks[i].z,2)))/quad.max_v;
			tasks[i].cost = kd*t_travel + kt*tasks[i].time - kp*tasks[i].priority - kl*tasks[i].lifeTime;
			if(tasks[i].cost < lowest_cost) {
				low_cost = i;
				lowest_cost = tasks[i].cost;
			}
		}
	}

	return low_cost;
}

/* ----- End Functions List ----- */

int main(int argc, char** argv) {
   
	ros::init(argc, argv, "task_coordinator");
	ros::NodeHandle nh;

	//Setup Timer
	timer_event = nh.createTimer(ros::Duration(15.0), timerCallback, false);
	life_event = nh.createTimer(ros::Duration(4.0), lifetimerCallback, false);
		
	// Grab Launch file parameters
	string quad_name, quad_frame, land_frame, world_frame;
	ros::param::get("~quad_name", quad_name);
	ros::param::get("~quad_frame", quad_frame);
	ros::param::get("~land_frame", land_frame);
	ros::param::get("~world_frame", world_frame);

	// Setup Publishers
	traj_pub = nh.advertise<pc_asctec_sim::pc_traj_cmd>(quad_name + "/traj_points",10);
	stop_pub = nh.advertise<std_msgs::Bool>(quad_name + "/stop",10);
	viz_pub = nh.advertise<visualization_msgs::Marker>(quad_name + "/land_line",10);
	bat_pub = nh.advertise<visualization_msgs::Marker>(quad_name + "/battery_ring", 10);
	task_pub = nh.advertise<visualization_msgs::Marker>(quad_name + "/tasks", 10);
	border_pub = nh.advertise<visualization_msgs::Marker>(quad_name + "/border", 10);

	// Setup Subscribers
	halt_sub = nh.subscribe(quad_name + "/halt",10,halt_callback);
	traj_sub = nh.subscribe(quad_name + "/traj_end", 10, traj_call);
	qb_sub = nh.subscribe(quad_name + "/battery", 10, qb_call);
	joy_sub = nh.subscribe("/joy",10, joy_callback);

	ros::Rate rate(freq);
	tf::StampedTransform transform;
	tf::TransformListener listener;
	ros::Duration(8.0).sleep();

        listener.waitForTransform(world_frame, quad_frame, ros::Time(0), ros::Duration(5.0));  //Uncomment with Vicon

	past = now = ros::Time().now();

	//Initialize Everything
	init_tasks();
	init(quad_frame, land_frame, world_frame);
	timer_event.start();
	ROS_INFO("Beginning Coordinator Node");

	while(ros::ok()) {	

		//Update Robot Positions

		//Crazyflie
		listener.lookupTransform(world_frame, quad_frame, ros::Time(0), transform);
		quad.x = transform.getOrigin().x();
		quad.y = transform.getOrigin().y();
		quad.z = transform.getOrigin().z();

		//Turtlebot
		listener.lookupTransform(t_bot.world, t_bot.frame, ros::Time(0), transform);

		//Time Update
		now = transform.stamp_;
		dt = (now.toSec() - past.toSec() + now.toNSec() - past.toNSec())/pow(10,9);
		past = now;

		t_bot.x = transform.getOrigin().x();
		t_bot.y = transform.getOrigin().y();
		t_bot.z = transform.getOrigin().z();
		t_bot.yaw = tf::getYaw(transform.getRotation());

		t_bot.vx = (t_bot.x - t_bot.xp)/dt;
		t_bot.vy = (t_bot.y - t_bot.yp)/dt;
		t_bot.vz = (t_bot.x - t_bot.zp)/dt;
		
		t_bot.xp = t_bot.x;
		t_bot.yp = t_bot.y;
		t_bot.zp = t_bot.z;

		//Battery, Border, and Tasks Visualization	
		float b_radius = (quad.battery - quad.b_min) * quad.v2d;
		show_battery(b_radius);
		show_border();
		show_tasks();

		if(st_mach.state == 0) {
			//Ascending to 0.0, 0.0, 1.0
			float tTravel = getTime(0.0, 0.0, 1.0) + 2;
			st_mach.traj = false;
			send_trajectory(tTravel, 2.0, 0.0, 0.0, 1.0);
			st_mach.state = 1;

		}else if(st_mach.state == 1) {
			//Attempt to complete tasks. If all tasks are completed, land
			if(st_mach.traj) {
				int current = setCosts();
				float tTravel = getTime(tasks[current].x, tasks[current].y, tasks[current].z);

				if((current != 0) && (current != 1)) {
					ROS_INFO("Completing task %i with cost %f", current, tasks[current].cost);
					send_trajectory(tTravel, tasks[current].time, tasks[current].x, tasks[current].y, tasks[current].z);
					st_mach.taskIs = current;
					st_mach.traj = false;

				}else if(current == 0) {
					ROS_INFO("Attempting to land on Turtlebot, cost: %f", tasks[current].cost);
					st_mach.state = 2;
					st_mach.taskIs = -1;

				}else if(current == 1) {
					ROS_INFO("Attempting to land at center, cost: %f", tasks[current].cost);
					st_mach.state = 5;
					st_mach.taskIs = -1;
				}	
			}

		}else if(st_mach.state == 2) {
			//Move to starting
			float tTravel1 = getTime(t_bot.x, t_bot.y, t_bot.z + 0.3);
			float pre_x = t_bot.x + t_bot.vx*(tTravel1 + 1);
			float pre_y = t_bot.y + t_bot.vy*(tTravel1 + 1);
			float tTravel2 = getTime(pre_x, pre_y, t_bot.z + 0.3);

			if(st_mach.traj) {
				ROS_INFO("Flying to Turtlebot, travel time: %f", tTravel1 + tTravel2);
				send_trajectory(tTravel1 + tTravel2, 1.0, pre_x + t_bot.vx * OFFSET * cos(t_bot.yaw), pre_y + t_bot.vy * OFFSET * sin(t_bot.yaw), t_bot.z + 0.3);
				st_mach.traj = false;
				st_mach.state = 3;	
			}

		}else if(st_mach.state == 3) {
			//Attempt to Land
			float tTravel = getTime(pre_x, pre_y, t_bot.z + 0.05);
			float pre_x = t_bot.x + t_bot.vx*tTravel;
			float pre_y = t_bot.y + t_bot.vy*tTravel;

			if(st_mach.traj) {
				send_trajectory(tLand, 0.0, pre_x + t_bot.vx * OFFSET * cos(t_bot.yaw), pre_y + t_bot.vy * OFFSET * sin(t_bot.yaw), t_bot.z + 0.05);
				st_mach.traj = false;
				st_mach.state = 4;
			}

		}else if(st_mach.state == 4) {
			//Kill Motors when close
			float xy_D = sqrt(pow((t_bot.x - quad.x),2) + pow((t_bot.y - quad.y),2));
			float z = quad.z - t_bot.z;

			if((xy_D < 0.1) && (z < 0.1)) {
				ROS_INFO("Killing Motors!");
				if(quad.battery < quad.b_min) {
					st_mach.state = 7;
					ROS_INFO("Charging to full...");

				}else {
					st_mach.state = 8;
					ROS_INFO("Waiting for new tasks...");

				}
				std_msgs::Bool stop_msg;
				stop_msg.data = true;
				stop_pub.publish(stop_msg);
			}
		}else if(st_mach.state == 5) {
			//Land at center
			float tTravel = getTime(0, 0, 0);

			if(st_mach.traj) {
				ROS_INFO("Flying to center, travel time: %f", tTravel);
				send_trajectory(tTravel, 0.0, 0.0, 0.0, 0.0);
				st_mach.traj = false;
				st_mach.state = 6;	
			}
		}else if(st_mach.state == 6) {
			//Kill Motors when close
			float xy_D = sqrt(pow(quad.x,2) + pow(quad.y,2));

			if((xy_D < 0.1) && (quad.z < 0.1)) {
				ROS_INFO("Killing Motors!");
				if(quad.battery < quad.b_min) {
					st_mach.state = 7;
					ROS_INFO("Charging to full...");

				}else {
					st_mach.state = 8;
					ROS_INFO("Waiting for new tasks...");

				}
				std_msgs::Bool stop_msg;
				stop_msg.data = true;
				stop_pub.publish(stop_msg);
			}
		}else if(st_mach.state == 7) {
			//Charge to full	
			if(quad.battery >= quad.b_max) {
				int cur_task = setCosts();
				ROS_INFO("Fully charged!");
				if(cur_task != -1 && cur_task != 0 && cur_task != 1 && !st_mach.ishalted) {
					st_mach.state = 1;
					std_msgs::Bool stop_msg;
					stop_msg.data = false;
					stop_pub.publish(stop_msg);

				}else {	
					ROS_INFO("Waiting for commands...");
					st_mach.state = 8;
				}
			}

		}else if(st_mach.state == 8) {
			//Charge while waiting for a new task
			int cur_task = setCosts();
			if(cur_task != -1 && cur_task != 0 && cur_task != 1 && !st_mach.ishalted) {
				ROS_INFO("Resuming task completion!");
				st_mach.state = 1;
				std_msgs::Bool stop_msg;
				stop_msg.data = false;
				stop_pub.publish(stop_msg);

			}else if(!st_mach.ishalted) {
				ROS_INFO("No tasks to complete!! Ignoring takeoff command");
			}
		}
		ros::spinOnce();
		rate.sleep();
	}
}