#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Joy.h>
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
bool waiting = false;
bool timing = false;

MatrixXd A(6,6);
double c_time = 0.0;
double range = 0.0001;
int point_ct;

ros::Publisher pos_goal, viz_goal;
ros::Subscriber goal_feedback, joy_feed, traj_feed, state_feed;

pc_asctec_sim::pc_state state_data;

void timerCallback(const ros::TimerEvent&) {
   ROS_INFO("Point %i timer expired", point_ct);
   point_ct++;
   c_time = 0.0;
   timing = false;
}

void state_callback(const pc_asctec_sim::pc_state::ConstPtr& msg) 
{
   state_data = *msg;
}

void calc_A(float time) 
{
   for(int i=5; i>=0; i--) {
      A(1,5-i) = pow(time,i);
      A(3,5-i) = i*pow(time,i-1);
      A(5,5-i) = i*(i-1)*pow(time,i-2);
   }
}

int main(int argc, char** argv) {
   
   ros::init(argc, argv, "pos_controller");
   ros::NodeHandle nh;
   ros::Timer timer_event = nh.createTimer(ros::Duration(3.0), timerCallback, true);
   timer_event.stop();

   string quad_name;
   string file = "avoid.txt";
   bool repeat = false;
   int rep_point;
   ros::param::get("~loop", repeat);
   ros::param::get("~loop_point", rep_point);
   ros::param::get("~name", quad_name);
   ros::param::get("~file", file);

   pos_goal = nh.advertise<pc_asctec_sim::pc_goal_cmd>(quad_name + "/pos_goals", 10);
   viz_goal = nh.advertise<geometry_msgs::PointStamped>(quad_name + "/viz_goals",10);
   state_feed = nh.subscribe(quad_name + "/state", 10, state_callback);
   ros::Rate rate(freq);

   ifstream scptfile;
   string line;
   string file_path = "/home/bezzo/catkin_ws/src/asctec_pos_control/pc_asctec_sim/trajectories/";
   file_path = file_path + file;
   cout << file_path << endl;
   scptfile.open(file_path.c_str());
   int num_points = -1;

   ROS_INFO("Opening trajectory file...");
   if(scptfile.is_open()) {
      while(!scptfile.eof()) {
	 getline(scptfile,line);

         if(line[0] != '!') {
	    num_points++;
	 }
      }
   }else {
      ROS_INFO("Error, trajectory points file could not be opened");
      ros::spin();
   }
   scptfile.clear();
   scptfile.seekg(0, scptfile.beg);

   MatrixXd B(24, num_points);
   MatrixXd X(24, num_points);
   MatrixXd t_data(2, num_points);

   int point = 0;
   while(!scptfile.eof()) {
      getline(scptfile,line);
      int l_cur = 0;
      int B_cur = 1;
      if(line[0] != '!') {
         int j = 0;
	 while(j < line.length()) {
	    if(line[j] != ' ') {
	       string temp = "";
	       int k = j;
	       while(line[k] != ' ') {
		  if(line[k] == 'a') {
		     break;
		  }
	          temp += line[k];
		  k++;
	       }
	       j = k;
	       if(l_cur < 2) {
	          t_data(l_cur, point) = strtof(temp.c_str(),0);
	       }else {
	          B(B_cur, point) = strtof(temp.c_str(),0);
	          B_cur += 2;
	       }
	       l_cur++;
	       if(B_cur == 25) {
	          j = line.length();
	       }
	    }
	    j++;
	 } 
         point++; 
      }
   }
   scptfile.close();
   for(int i = 0; i < 6; i++) {
      for(int k = 0; k < 6; k++) {
         A(i,k) = 0.0;
      }
   }
   A(0,5) = 1;
   A(2,4) = 1;
   A(4,3) = 2; 

   for(int i = 0; i < 12; i++) {
      B(2*i,0) = 0.0;
   }

   ros::Duration(4.0).sleep();
   ros::spinOnce();

   B(0,0) = state_data.x;
   B(6,0) = state_data.y;
   B(12,0) = state_data.z;
   B(18,0) = state_data.yaw;

   for(int i = 1; i < num_points; i++) {
      for(int j = 0; j < 4; j++) {
         B(6*j,i) = B(6*j+1,i-1);
         B(6*j+2,i) = B(6*j+3,i-1);
         B(6*j+4,i) = B(6*j+5,i-1);      
      }
   }  

   ROS_INFO("Parsed Trajectory Points! Calculating Trajectory...");
   ros::Time t_s = ros::Time::now();
   for(int i = 0; i < num_points; i++) {
      calc_A(t_data(1,i));
      for(int k = 0; k < 4; k++) {
         X.block<6,1>(6*k,i) = A.colPivHouseholderQr().solve(B.block<6,1>(6*k,i));
      }
   }
   ros::Time t_e = ros::Time::now();
   float t_f = ((t_e - t_s).toNSec());
   ROS_INFO("Trajectory with %i points calculated. Time of calculation (ms): %f", num_points, t_f/1000000);

   point_ct = 0;
   ROS_INFO("Starting Trajectory!!");

   while (ros::ok()) {
      ros::spinOnce();
      if(point_ct == num_points && !waiting) {
	    if(!repeat) {
	       waiting = true;
	    }else {
	       point_ct = rep_point;
	    }
	    ROS_INFO("Trajectory Completed!");
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

	 pos_goal.publish(goal);
         geometry_msgs::PointStamped viz;
         viz.header.stamp = ros::Time::now();
         viz.header.frame_id = "/odom";
         viz.point.x = goal.x;
         viz.point.y = goal.y;
         viz.point.z = goal.z;
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
