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
#define TASKS 15
#define START_TASKS 8
#define TASK_TIME 5
#define TASK_PRI 10
#define BATTERY_BUFFER 8

#define kp 22.0
#define kt 8.0
#define kd 12.0
#define kl 1.0
#define kb 410.0 
#define kdb 17.0

#define BORDERX 1.2
#define BORDERY 1.8
#define MAX_V 0.4
#define MAX_LAND_V 0.3
#define Qvd MAX_V*210

#define MAX_T (sqrt(4*BORDERX*BORDERX + 4*BORDERY*BORDERY)+1)/MAX_V

using namespace std;

ros::Publisher traj_pub, viz_pub, stop_pub, bat_pub, base_pub, task_pub, border_pub, path_pub;
ros::Subscriber halt_sub, traj_sub, qb_sub, tb_sub, joy_sub;
ros::Timer timer_event;
ros::Timer life_event;

double dt = 0.0; 
ros::Time past, now;

float tLand = 1;

struct sM {
	bool ishalted;
	bool traj;
	bool started;
	int state;
	int taskIs;

	int path[TASKS];
	bool search[TASKS];
	int p_now;
	int p_end;
	float cost_arr[TASKS][TASKS];
	
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
	float b_points[BATTERY_BUFFER];
	int b_count;
	float battery;
	float v2t;
	float b_min;
	float b_max;
	float battery_low;

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

/* ------------------------------------------------------------ Callbacks ------------------------------------------------------------ */
void joy_callback(const sensor_msgs::Joy::ConstPtr& msg)
{
	if(msg->buttons[0]) {
		ROS_INFO("External start joy-command called!");
		st_mach.ishalted = false;
		st_mach.started = true;

	}else if(msg->buttons[1] && st_mach.started) {
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
	quad.b_count &= (BATTERY_BUFFER-1);
	quad.battery = 0.0;
	for(int i = 0; i < BATTERY_BUFFER; i++) {
		quad.battery += quad.b_points[i];
	}
	quad.battery = quad.battery / BATTERY_BUFFER;
	if(quad.battery < quad.battery_low) {
		quad.battery_low = quad.battery;
	}
}

void tb_call(const std_msgs::Float32::ConstPtr& msg)
{
	t_bot.b_points[t_bot.b_count] = msg->data;
	t_bot.b_count++;
	t_bot.b_count &= (BATTERY_BUFFER-1);
	t_bot.battery = 0.0;
	for(int i = 0; i < BATTERY_BUFFER; i++) {
		t_bot.battery += t_bot.b_points[t_bot.b_count];
	}
	t_bot.battery = t_bot.battery / BATTERY_BUFFER;
}

void timerCallback(const ros::TimerEvent&) 
{
	int newTask = -1;
	for(int i = 3; i < TASKS; i++) {
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
		tasks[newTask].isDone = false;
		ROS_INFO("New Task added at x: %f, y: %f", tasks[newTask].x, tasks[newTask].y);
		if(tasks[1].isDone) {
			st_mach.state = 0;
		}
	}else {
		ROS_INFO("Task Queue Full!!");

	}
}

void lifetimerCallback(const ros::TimerEvent&) 
{
	for(int i = 2; i < TASKS; i++) {
		if(!tasks[i].isDone) {
			if(tasks[i].cost > 1) {
				tasks[i].lifeTime++;
			}
		}
	}
}

/* ------------------------------------------------------------ Inits ------------------------------------------------------------ */
void init(string quad_str, string turtle_str, string world_str) 
{
	quad.frame = quad_str;
	quad.world = world_str;
	quad.x = quad.y = quad.z = 0.0;
	quad.max_v = MAX_V;

	quad.v2t = 12;
	quad.battery = quad.battery_low = 4.0;
	quad.b_min = 3.15;
	quad.b_max = 4.0;

	for(int i = 0; i < BATTERY_BUFFER; i++) {
		quad.b_points[i] = 4.0;
	}
	quad.b_count = 0;
	
	t_bot.frame = turtle_str;
	t_bot.world = world_str;
	t_bot.x = t_bot.y = t_bot.z = 0.0;
	t_bot.xp = t_bot.yp = t_bot.zp = 0.0;

	t_bot.battery = 4.0;
	for(int i = 0; i < BATTERY_BUFFER; i++) {
		t_bot.b_points[i] = 4.0;
	}
	t_bot.b_count = 0;


	st_mach.ishalted = false;
	st_mach.traj = false;
	st_mach.started = false;
	st_mach.state = -2;

}

void init_demo_tasks(void)
{
	tasks[0].isDone = true;
	tasks[1].isDone = true;

	float x[4] = {-1.2, -0.9, 1.1, 0.3};
	float y[4] = {-1.6, 1.3, 0.4, -0.2};
	float pri[4] = {2, 3, 9, 7}; 
	float time[4] = {2, 2, 4, 3};
	
	for(int i = 0; i < 4; i++) {
		tasks[i+3].x = x[i];
		tasks[i+3].y = y[i];
		tasks[i+3].z = 1.0;
		tasks[i+3].time = time[i];
		tasks[i+3].lifeTime = 0;
		tasks[i+3].priority = pri[i];
		tasks[i+3].isDone = false;
	}

	for(int i = 7; i < TASKS; i++) {
		tasks[i].isDone = true;
	}
}

void init_tasks(void)
{
	tasks[0].isDone = true;
	tasks[1].isDone = true;

	for(int i = 3; i < START_TASKS; i++) {
		tasks[i].x = float(rand() % int(2000*BORDERX)) / 1000 - BORDERX;
		tasks[i].y = float(rand() % int(2000*BORDERY)) / 1000 - BORDERY;
		tasks[i].z = 1.0;
		tasks[i].time = rand() % TASK_TIME + 1;
		tasks[i].lifeTime = 0;
		tasks[i].priority = rand() % 10 + 1;
		tasks[i].isDone = false;
	}

	for(int i = START_TASKS; i < TASKS; i++) {
		tasks[i].isDone = true;
	}
}

/* ------------------------------------------------------------ Viz Functions ------------------------------------------------------------ */
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
		vis_ring.x = cos(i) * radius + quad.x;
		vis_ring.y = sin(i) * radius + quad.y;
		vis_ring.z = quad.z;
		ring.points.push_back(vis_ring);
		i += 0.05;

		vis_ring.x = cos(i) * radius + quad.x;
		vis_ring.y = sin(i) * radius + quad.y;
		vis_ring.z = quad.z;
		ring.points.push_back(vis_ring);
	}

	bat_pub.publish(ring);
}

void show_base_range(float radius)
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
	ring.color.b = 1.0;

	ring.scale.x = 0.05;
	ring.scale.y = 0.05;

	float i = 0.0;
	while(i < 2*M_PI) {
		vis_ring.x = cos(i) * radius + t_bot.x;
		vis_ring.y = sin(i) * radius + t_bot.y;
		vis_ring.z = t_bot.z;
		ring.points.push_back(vis_ring);
		i += 0.05;

		vis_ring.x = cos(i) * radius + t_bot.x;
		vis_ring.y = sin(i) * radius + t_bot.y;
		vis_ring.z = t_bot.z;
		ring.points.push_back(vis_ring);
	}

	base_pub.publish(ring);
}

void show_optimal_path(void)
{
	visualization_msgs::Marker path;
	geometry_msgs::Point path_point;

	path.header.frame_id = quad.world;
	path.header.stamp = ros::Time::now();
	path.id = 2;
	path.action = visualization_msgs::Marker::ADD;
	path.type = visualization_msgs::Marker::LINE_LIST;
	path.color.a = 1.0;
	path.color.g = 1.0;				
	path.color.b = 1.0;

	path.scale.x = 0.05;
	path.scale.y = 0.05;

	int i = st_mach.p_end;
	while(i > 1) {
		path_point.x = tasks[st_mach.path[i]].x;
		path_point.y = tasks[st_mach.path[i]].y;
		path_point.z = tasks[st_mach.path[i]].z;
		path.points.push_back(path_point);
		i--;

		path_point.x = tasks[st_mach.path[i]].x;
		path_point.y = tasks[st_mach.path[i]].y;
		path_point.z = tasks[st_mach.path[i]].z;
		path.points.push_back(path_point);
	}
	path_pub.publish(path);
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

	corner.x = -BORDERX;
	corner.y = -BORDERY;	
	corner.z = 1.0;
	border.points.push_back(corner);

	corner.x = -BORDERX;
	corner.y = BORDERY;	
	corner.z = 1.0;
	border.points.push_back(corner);

	corner.x = -BORDERX;
	corner.y = BORDERY;	
	corner.z = 1.0;
	border.points.push_back(corner);

	corner.x = BORDERX;
	corner.y = BORDERY;	
	corner.z = 1.0;
	border.points.push_back(corner);

	corner.x = BORDERX;
	corner.y = BORDERY;	
	corner.z = 1.0;
	border.points.push_back(corner);

	corner.x = BORDERX;
	corner.y = -BORDERY;	
	corner.z = 1.0;
	border.points.push_back(corner);

	corner.x = BORDERX;
	corner.y = -BORDERY;	
	corner.z = 1.0;
	border.points.push_back(corner);

	corner.x = -BORDERX;
	corner.y = -BORDERY;	
	corner.z = 1.0;
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

	task_list.scale.x = 0.1;
	task_list.scale.y = 0.1;

	for(int i = 1; i < TASKS; i++) {
		if(!tasks[i].isDone) {
			task.x = tasks[i].x;
			task.y = tasks[i].y;
			task.z = tasks[i].z;
			task_list.points.push_back(task);
		}
	}

	task_pub.publish(task_list);
}

/* ------------------------------------------------------------ Pub Functions ------------------------------------------------------------ */

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

/* ------------------------------------------------------------ General Functions ------------------------------------------------------------ */

float getTime(float x, float y, float z) 
{
	float d_target = sqrt(pow(x - quad.x,2) + pow(y - quad.y,2) + pow(z - quad.z,2));
	return d_target/quad.max_v;
}

void printPath(void)
{
	cout << endl;

	/*for(int i = 1; i < TASKS; i++) {
		if(!tasks[i].isDone) {
			cout << "Task: " << i << " ";
			for(int j = 0; j < TASKS; j++) {
				cout << st_mach.cost_arr[i][j] << " ";
			}
			cout << endl;
		}
	}

	cout << endl;
	*/
	float costOf = 0;
	int past = 2;

	for(int i = st_mach.p_end -1; i > 0; i--) {
		costOf += st_mach.cost_arr[past][st_mach.path[i]];
		past = st_mach.path[i];
	}
	cout << "Path: ";
	for(int i = st_mach.p_end; i > 0; i--) {
		cout << st_mach.path[i] << " ";	
	}
	cout << "Cost: " << costOf << endl;
}

void calcCosts(int task)
{
	for(int i = 2; i < TASKS; i++) {
		if(!tasks[i].isDone) {
			float t_travel = (sqrt(pow(tasks[task].x - tasks[i].x,2) + pow(tasks[task].y - tasks[i].y,2) + pow(tasks[task].z - tasks[i].z,2)))/quad.max_v;
			st_mach.cost_arr[task][i] = kd*(t_travel) + kt*tasks[i].time - kp*tasks[i].priority - kl*tasks[i].lifeTime;
		}
	}
}

void getCosts(void)
{
	float t_center = (sqrt(pow(quad.x,2) + pow(quad.y,2) + pow(quad.z,2)))/quad.max_v;
	float t_turtle = (sqrt(pow(quad.x - t_bot.x,2) + pow(quad.y - t_bot.y,2) + pow(quad.z - t_bot.z,2)))/quad.max_v;

	tasks[0].cost = kdb*(t_center + tLand) - kb*(quad.b_max - quad.battery_low);
	tasks[1].cost = kdb*(t_turtle + tLand) - kb*(quad.b_max - quad.battery_low);

	for(int i = 2; i < TASKS; i++) {
		if(!tasks[i].isDone) {
			calcCosts(i);
		}
	}
}

int getLowCost(int task)
{
	float low_cost = 9000;
	int low_val = -1;

	for(int i = 1; i < TASKS; i++) {
		if(st_mach.search[i]) {
			if(low_cost > st_mach.cost_arr[task][i]) {
				low_cost = st_mach.cost_arr[task][i];
				low_val = i;
			}
		}
	}
	return low_val;
}

bool pathCost(int count, int start_task)
{
	int min_val = getLowCost(start_task);
	bool isGood = false;
	if(min_val != -1) {
		st_mach.path[count] = min_val;	
		st_mach.search[min_val] = false;
		count--;
		if(count == 0) {
			isGood = true;
		}else {
			isGood = pathCost(count, min_val);
		}
	}
	return isGood;
}

bool setPath(int end) 
{
	int valid_tasks = 0;
	bool isFound = false;

	st_mach.search[0] = false;
	st_mach.search[1] = false;
	ros::Time now = ros::Time(0);

	for(int i = 1; i < TASKS; i++) {
		st_mach.search[i] = false;
		if(!tasks[i].isDone) {
			valid_tasks++;
			st_mach.search[i] = true;
		}	
	}
	st_mach.p_end = valid_tasks;

	if(end == -1) {
		if(pathCost(valid_tasks, 2)) {
			ros::Duration total = now - ros::Time(0);
			ROS_INFO("Optimal path found!");
			isFound = true;
		}
	}else {
		if(end <= valid_tasks) {

		}else {

		}
	}
	return isFound;
}

bool isComplete(void) 
{
	bool done = true;
	for(int i = 3; i < TASKS; i++) {
		if(!tasks[i].isDone) {
			done = false;
		}
	}
	return done;
}

void setStart(void)
{
	tasks[2].x = quad.x;
	tasks[2].y = quad.y;
	tasks[2].z = quad.z;
	tasks[2].isDone = false;
	tasks[2].priority = 20;
	tasks[2].time = 0;
}

/* ------------------------------------------------------------ End Functions List ------------------------------------------------------------ */

int main(int argc, char** argv) {
   
	ros::init(argc, argv, "task_coordinator");
	ros::NodeHandle nh;

	//Setup Timer
	timer_event = nh.createTimer(ros::Duration(25.0), timerCallback, false);
	timer_event.stop();
	life_event = nh.createTimer(ros::Duration(10.0), lifetimerCallback, false);
		
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
	base_pub = nh.advertise<visualization_msgs::Marker>(quad_name + "/base_ring", 10);
	task_pub = nh.advertise<visualization_msgs::Marker>(quad_name + "/tasks", 10);
	border_pub = nh.advertise<visualization_msgs::Marker>(quad_name + "/border", 10);
	path_pub = nh.advertise<visualization_msgs::Marker>(quad_name + "/path", 10);

	// Setup Subscribers
	halt_sub = nh.subscribe(quad_name + "/halt",10,halt_callback);
	traj_sub = nh.subscribe(quad_name + "/traj_end", 10, traj_call);
	qb_sub = nh.subscribe(quad_name + "/battery", 10, qb_call);
	joy_sub = nh.subscribe("/joy",10, joy_callback);

	ros::Rate rate(freq);
	tf::StampedTransform transform;
	tf::TransformListener listener;
	ros::Duration(7.0).sleep();

        listener.waitForTransform(world_frame, quad_frame, ros::Time(0), ros::Duration(5.0));  //Uncomment with Vicon

	past = now = ros::Time().now();

	//Initialize Everything
	init_tasks();
	//init_demo_tasks();
	init(quad_frame, land_frame, world_frame);

	ROS_INFO("Coordinator ready...");
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

		//Battery, Border, Tasks, and Path Visualization	
		float b_radius = (quad.battery_low - quad.b_min) * quad.v2t;
		float base_rad = sqrt(pow(quad.x - t_bot.x,2) + pow(quad.y - t_bot.y,2));

		show_battery(b_radius);
		show_base_range(base_rad/MAX_LAND_V);
		show_border();
		show_tasks();
		show_optimal_path();
		
		if(st_mach.state == -2) {
			//Wait for start signal
			if(st_mach.started) {
				ROS_INFO("Beginning Coordinator Node");
				st_mach.state = -1;
				std_msgs::Bool stop_msg;
				stop_msg.data = false;
				stop_pub.publish(stop_msg);
			}

		}else if(st_mach.state == -1) {
			//Ascending to 0.0, 0.0, 1.0
			float tTravel = getTime(0, 0, 1.0);
			st_mach.traj = false;
			send_trajectory(tTravel, 2.0, 0.0, 0.0, 1.0);
			st_mach.state = 0;

		}else if(st_mach.state == 0) {
			//Compute Fastest Path through task list
			setStart();
			getCosts();
			timer_event.start();

			if(setPath(-1)) {
				st_mach.p_now = st_mach.p_end - 1;
				st_mach.state = 1;
				printPath();
			}else {
				ROS_INFO("Error Computing Path! Hovering and waiting...");
				ros::spin();
			}

		}else if(st_mach.state == 1) {
			//Attempt to complete tasks. If all tasks are completed, land
			if(st_mach.traj && st_mach.p_now != 0) {
				float tTravel = getTime(tasks[st_mach.path[st_mach.p_now]].x, tasks[st_mach.path[st_mach.p_now]].y, tasks[st_mach.path[st_mach.p_now]].z);				
				ROS_INFO("Completing task %i", st_mach.path[st_mach.p_now]);
				send_trajectory(tTravel, tasks[st_mach.path[st_mach.p_now]].time, tasks[st_mach.path[st_mach.p_now]].x, tasks[st_mach.path[st_mach.p_now]].y, tasks[st_mach.path[st_mach.p_now]].z);
				st_mach.taskIs = st_mach.path[st_mach.p_now];
				st_mach.traj = false;		
				st_mach.p_now--;

				if(st_mach.taskIs == 1) {
					st_mach.state == 2;
				}

				if(quad.battery_low < quad.b_min && tasks[1].isDone) {
					ROS_INFO("Battery Depleted, planning optimal route with charging");
					tasks[1].isDone = false;
					st_mach.state = 0;
				}

			}else if(st_mach.p_now == 0) {
				ROS_INFO("Tasks completed, waiting on charge station...");
				st_mach.state = 2;

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
			float tTravel = getTime(t_bot.x, t_bot.y, t_bot.z);
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

			if((xy_D < 0.1) && (z < 0.05)) {
				ROS_INFO("Killing Motors!");
				tasks[1].isDone = true;
				if(quad.battery_low < quad.b_min) {
					st_mach.state = 8;
					ROS_INFO("Charging to full...");

				}else {
					st_mach.state = 9;
					ROS_INFO("Waiting for new tasks...");

				}
				std_msgs::Bool stop_msg;
				stop_msg.data = true;
				stop_pub.publish(stop_msg);
				ros::Duration(3.0).sleep();

			}else if((xy_D > 0.1) && (z < 0.1)) {
				ROS_INFO("Missed Target! Trying again...");
				st_mach.traj = false;
				st_mach.state = 3;
			}

		}else if(st_mach.state == 5) {
			//Fly to center
			float tTravel = getTime(0, 0, 1);

			if(st_mach.traj) {
				ROS_INFO("Flying to center, travel time: %f", tTravel);
				send_trajectory(tTravel, 0.0, 0.0, 0.0, 1.0);
				st_mach.traj = false;
				st_mach.state = 6;	
			}
		}else if(st_mach.state == 6) {
			//Land at center
			float tTravel = getTime(0, 0, 0);

			if(st_mach.traj) {
				ROS_INFO("Landing at center, travel time: %f", tTravel);
				send_trajectory(tTravel, 2.0, 0.0, 0.0, 1.0);
				st_mach.traj = false;
				st_mach.state = 7;	
			}
		}else if(st_mach.state == 7) {
			//Kill Motors when close
			float xy_D = sqrt(pow(quad.x,2) + pow(quad.y,2));

			if((xy_D < 0.1) && (quad.z < 0.1)) {
				ROS_INFO("Killing Motors!");
				if(quad.battery_low < quad.b_min) {
					st_mach.state = 8;
					ROS_INFO("Charging to full...");

				}else {
					st_mach.state = 9;
					ROS_INFO("Waiting for new tasks...");

				}
				std_msgs::Bool stop_msg;
				stop_msg.data = true;
				stop_pub.publish(stop_msg);
				ros::Duration(3.0).sleep();
			}

		}else if(st_mach.state == 8) {
			//Charge to full	
			if(quad.battery >= quad.b_max) {
				ROS_INFO("Fully charged!");
				if(!isComplete() && !st_mach.ishalted) {
					st_mach.state = 1;
					std_msgs::Bool stop_msg;
					stop_msg.data = false;
					stop_pub.publish(stop_msg);

				}else {	
					ROS_INFO("Waiting for commands...");
					st_mach.state = 9;
				}
			}

		}else if(st_mach.state == 9) {
			//Charge while waiting for a new task
			if(!isComplete() && !st_mach.ishalted) {
				ROS_INFO("Resuming task completion!");
				st_mach.state = 1;
				std_msgs::Bool stop_msg;
				stop_msg.data = false;
				stop_pub.publish(stop_msg);
				send_trajectory(4, 1, quad.x, quad.y, 1.0);
				st_mach.traj = false;

			}
		}
		ros::spinOnce();
		rate.sleep();
	}
}
