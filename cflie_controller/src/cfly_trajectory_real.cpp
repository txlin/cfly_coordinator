#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/PointStamped.h>
#include <pc_asctec_sim/pc_goal_cmd.h>
#include <pc_asctec_sim/pc_feedback.h>
#include <pc_asctec_sim/pc_traj_cmd.h>
#include <pc_asctec_sim/pc_state.h>
#include <math.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <Eigen/Dense>

#define freq 10.0
#define dt 1/freq

using namespace std;
using Eigen::MatrixXd;

int yaw_counter = 0;
bool waiting = true;
bool timing = false;

MatrixXd A(6,6);
MatrixXd B(24,24);
MatrixXd X(24,24);
MatrixXd t_data(2,24);

double c_time = 0.0;
double range = 0.0001;
int point_ct = 0;
int max_point = 0;

ros::Publisher pos_goal, traj_goal, viz_goal;
ros::Subscriber goal_feedback, joy_feed, traj_feed, state_feed;

pc_asctec_sim::pc_state state_data;

void init_A(void)
{
	for(int i = 0; i < 6; i++) {
		for(int k = 0; k < 6; k++) {
			A(i,k) = 0.0;
		}
	}

	A(0,5) = 1;
	A(2,4) = 1;
	A(4,3) = 2; 
}

void calc_A(float time) 
{
	for(int i=5; i>=0; i--) {
		A(1,5-i) = pow(time,i);
		A(3,5-i) = i*pow(time,i-1);
		A(5,5-i) = i*(i-1)*pow(time,i-2);
	}
}



void timerCallback(const ros::TimerEvent&) {
	ROS_INFO("Point %i timer expired", point_ct);
	point_ct++;
	c_time = 0.0;
	timing = false;
}

void traj_callback(const pc_asctec_sim::pc_traj_cmd::ConstPtr& msg) 
{
	if(waiting) {
		if(msg->points != 0) {
			max_point = msg->points;
			for(int i = 0; i < max_point; i++) {
				B(1,i) = msg->x_end[i];
				B(3,i) = msg->x_v_end[i];
				B(5,i) = msg->x_a_end[i];
	
				B(7,i) = msg->y_end[i];
				B(9,i) = msg->y_v_end[i];
				B(11,i) = msg->y_a_end[i];

				B(13,i) = msg->z_end[i];
				B(15,i) = msg->z_v_end[i];
				B(17,i) = msg->z_a_end[i];

				B(19,i) = msg->yaw_end[i];
				B(21,i) = msg->yaw_v_end[i];
				B(23,i) = msg->yaw_a_end[i];

				t_data(0,i) = msg->wait_time[i];
				t_data(1,i) = msg->duration[i];

			}
			B(0,0) = state_data.x;
			B(2,0) = state_data.x_vel;
			B(4,0) = state_data.x_acc;
	
			B(6,0) = state_data.y;
			B(8,0) = state_data.y_vel;
			B(10,0) = state_data.y_acc;
	
			B(12,0) = state_data.z;
			B(14,0) = state_data.z_vel;
			B(16,0) = state_data.z_acc;
		
			B(18,0) = state_data.yaw;
			B(20,0) = state_data.yaw_vel;
			B(22,0) = state_data.yaw_acc;
	
			for(int i = 1; i < max_point; i++) {
				for(int j = 0; j < 4; j++) {
					B(6*j,i) = B(6*j+1,i-1);
					B(6*j+2,i) = B(6*j+3,i-1);
					B(6*j+4,i) = B(6*j+5,i-1);      
				}
			} 
		
			point_ct = 0.0;
			c_time = 0.0;
			ROS_INFO("New Trajectory with %i points heard!", msg->points);
			ros::Time t_s = ros::Time::now();
			for(int i = 0; i < max_point; i++) {
				calc_A(t_data(1,i));
				for(int k = 0; k < 4; k++) {
					X.block<6,1>(6*k,i) = A.colPivHouseholderQr().solve(B.block<6,1>(6*k,i));
				}
			}
			ros::Time t_e = ros::Time::now();
			float t_f = ((t_e - t_s).toNSec());
			ROS_INFO("Trajectory calculation took %f ms", t_f/1000000);
			waiting = false;
		}else {
			ROS_INFO("Error, trajectory has 0 points. Msg ignored");
		}
	}else {
		ROS_INFO("Trajectory already in motion! Msg ignored");
	}
}

void state_callback(const pc_asctec_sim::pc_state::ConstPtr& msg) 
{
	state_data = *msg;
}

int main(int argc, char** argv) {
   
	ros::init(argc, argv, "pos_controller");
	ros::NodeHandle nh;
	ros::Timer timer_event = nh.createTimer(ros::Duration(3.0), timerCallback, true);
	timer_event.stop();

	string quad_name;
	ros::param::get("~name", quad_name);

	pos_goal = nh.advertise<pc_asctec_sim::pc_goal_cmd>(quad_name + "/pos_goals", 10);
	viz_goal = nh.advertise<geometry_msgs::PointStamped>(quad_name + "/viz_goals",10);
	traj_goal = nh.advertise<std_msgs::Empty>(quad_name + "/traj_end",10);
	state_feed = nh.subscribe(quad_name + "/state", 10, state_callback);
	traj_feed = nh.subscribe(quad_name + "/traj_points", 10, traj_callback);
	ros::Rate rate(freq);

	ros::spinOnce();
	ros::Duration(5.0).sleep();
	init_A();
	while (ros::ok()) {
		ros::spinOnce();
		if(point_ct == max_point && !waiting) {
			ROS_INFO("Trajectory Completed!");
			waiting = true;
			std_msgs::Empty fin;
			traj_goal.publish(fin);

			c_time = 0.0;
		}
		if(!waiting) {
			pc_asctec_sim::pc_goal_cmd goal; 

			if(!timing) {
				c_time += dt;
			}

			for(int i=5; i>=0; i--) {
				goal.x += X(5-i,point_ct)*pow(c_time,i);
				goal.x_vel += i*X(5-i,point_ct)*pow(c_time,i-1);
				goal.x_acc += i*(i-1)*X(5-i,point_ct)*pow(c_time,i-2);

				goal.y += X(11-i,point_ct)*pow(c_time,i);
				goal.y_vel += i*X(11-i,point_ct)*pow(c_time,i-1);
				goal.y_acc += i*(i-1)*X(11-i,point_ct)*pow(c_time,i-2);

				goal.z += X(17-i,point_ct)*pow(c_time,i);
				goal.z_vel += i*X(17-i,point_ct)*pow(c_time,i-1);
				goal.z_acc += i*(i-1)*X(17-i,point_ct)*pow(c_time,i-2);

				goal.yaw += X(23-i,point_ct)*pow(c_time,i);
				goal.yaw_vel += i*X(23-i,point_ct)*pow(c_time,i-1);
				goal.yaw_acc += i*(i-1)*X(23-i,point_ct)*pow(c_time,i-2);
			}
			goal.goal_id = c_time;

			geometry_msgs::PointStamped viz;
			viz.header.stamp = ros::Time::now();
			viz.header.frame_id = "/odom";
			viz.point.x = goal.x;
			viz.point.y = goal.y;
			viz.point.z = goal.z;

			pos_goal.publish(goal);
			viz_goal.publish(viz);

			if((c_time + range) >= t_data(1,point_ct)) {
				if(t_data(0,point_ct) != 0.0 && !timing) {
					timing = true;
					timer_event.setPeriod(ros::Duration(t_data(0,point_ct)), true);
					ROS_INFO("Point %i reached, waiting for %f seconds", point_ct, t_data(0,point_ct));
					timer_event.start();

				}else if(t_data(0,point_ct) == 0.0 && !timing){
					ROS_INFO("Point %i reached", point_ct);
					point_ct++;
					c_time = 0.0;
					timer_event.stop();
				}
			}
		}
		rate.sleep();
	}
	return 0;
}
