#include <cflie_controller.h>
	
geometry_msgs::Twist real_cmd;
pc_asctec_sim::pc_feedback on_goal;
pc_asctec_sim::pc_state state_data;

PID_DATA ctl_data;
POS_DATA position_data;

ros::Time past, now;

double dt = 0.0;

float battery = 4.0;
int battery_status = 4;

float k_p_xy, k_i_xy, k_d_xy, k_a_xy;
float k_p_z, k_i_z, k_d_z, k_a_z;
float k_p_yaw, k_i_yaw, k_d_yaw, k_a_yaw;

bool steady = false;
bool counting = false;
bool stopped = false;


float final_x, final_y, final_z, final_yaw;
int yaw_counter = 0;
string name;

ros::Publisher goal_feedback, accel_quad_cmd, state_pub;
ros::Subscriber pc_goal_cmd, bat_sub, joy_call, stop_sub;

void init(struct POS_DATA * pos_ptr, struct PID_DATA * ctl_ptr)
{
   //initialize controller
   ctl_ptr->error_x = 0.0;
   ctl_ptr->integral_x = 0.0;
   ctl_ptr->error_x_vel = 0.0;

   ctl_ptr->error_y = 0.0;
   ctl_ptr->integral_y = 0.0;
   ctl_ptr->error_y_vel = 0.0;

   ctl_ptr->error_z = 0.0;
   ctl_ptr->integral_z = 0.0;
   ctl_ptr->error_z_vel = 0.0;

   ctl_ptr->error_yaw = 0.0;
   ctl_ptr->integral_yaw = 0.0;
   ctl_ptr->error_yaw_vel = 0.0;
   
   //initialize position data
   pos_ptr->pos_x = 0.0;
   pos_ptr->pos_y = 0.0;
   pos_ptr->pos_z = 0.0;
   pos_ptr->pos_yaw = 0.0;

   pos_ptr->pos_x_past = 0.0;
   pos_ptr->pos_y_past = 0.0;
   pos_ptr->pos_z_past = 0.0;
   pos_ptr->pos_yaw_past = 0.0;

   pos_ptr->vel_x = 0.0;
   pos_ptr->vel_y = 0.0;
   pos_ptr->vel_z = 0.0;
   pos_ptr->vel_yaw = 0.0;

   pos_ptr->goal_x = 0.0;
   pos_ptr->goal_y = 0.0;
   pos_ptr->goal_z = 0.0;
   pos_ptr->goal_yaw = 0.0;

   pos_ptr->goal_vel_x = 0.0;
   pos_ptr->goal_vel_y = 0.0;
   pos_ptr->goal_vel_z = 0.0;
   pos_ptr->goal_vel_yaw = 0.0;

   pos_ptr->goal_range = 0.05;
   pos_ptr->wait_time = 0.0;
   pos_ptr->waiting = false;
   pos_ptr->goal_arrival = false;
   pos_ptr->goal_id = "Init";
}

bool setBatteryStatus(void) 
{
   bool isGood = true;
   ROS_INFO("Starting Battery: %f V", battery);
   if(battery >= BATTERY_FULL) {
      battery_status = 4;
   }else if(battery >= BATTERY_MID) {
      battery_status = 3;
   }else if(battery > BATTERY_EMPTY) {
      battery_status = 2;
   }else if(battery <= BATTERY_EMPTY) {
      ROS_INFO("Cannot Fly, Battery is too low!!");
      isGood = false;
   }
   return isGood;
}

bool goal_arrived(struct PID_DATA * ctl_ptr, struct POS_DATA * pos_ptr)
{
   float distance = (ctl_ptr->error_x) * (ctl_ptr->error_x) + 
                           (ctl_ptr->error_y) * (ctl_ptr->error_y) + 
                           (ctl_ptr->error_z) * (ctl_ptr->error_z); 
   
   if((distance <= (pos_ptr->goal_range)) && not (pos_ptr->goal_arrival)) {
      pos_ptr->wait_start = ros::Time::now().toSec(); 
      pos_ptr->waiting = true;  
      pos_ptr->goal_arrival = true;
   }
   if(pos_ptr->waiting) {
      if((ros::Time::now().toSec() - pos_ptr->wait_start) >= pos_ptr->wait_time) {
         on_goal.goal_id = pos_ptr->goal_id;
         on_goal.event = ros::Time::now();
         goal_feedback.publish(on_goal);
         pos_ptr->waiting = false;
         return true;
      }
   }
   return false;
}

void check_bat(void) {
	
	if(battery <= BATTERY_MID && battery_status == 3) {
		ROS_INFO("50 Percent Battery Remaining: %f V", battery);
		battery_status = 2;	
	}else if(battery <= BATTERY_EMPTY && battery_status == 2) {
		ROS_INFO("10 Percent Battery Remaining: %f V, land now!!", battery);
		battery_status = 1;
	}
}

void callback(cflie_controller::tunerConfig &config, uint32_t level) {

	k_p_xy = config.k_p_xy;
	k_i_xy = config.k_i_xy;
	k_d_xy = config.k_d_xy;
	k_a_xy = config.k_a_xy;

	k_p_z = config.k_p_z;
	k_i_z = config.k_i_z;
	k_d_z = config.k_d_z;
	k_a_z = config.k_a_z;

	k_p_yaw = config.k_p_yaw;
	k_i_yaw = config.k_i_yaw;
	k_d_yaw = config.k_d_yaw;
	k_a_yaw = config.k_a_yaw;
}

void goal_callback(const pc_asctec_sim::pc_goal_cmd::ConstPtr& msg)
{

       position_data.goal_x = msg->x;
       position_data.goal_y = msg->y;
       position_data.goal_z = msg->z;
       position_data.goal_yaw = msg->yaw;

       position_data.goal_vel_x = msg->x_vel;
       position_data.goal_vel_y = msg->y_vel;
       position_data.goal_vel_z = msg->z_vel;
       position_data.goal_vel_yaw = msg->yaw_vel;

       position_data.goal_acc_x = msg->x_acc;
       position_data.goal_acc_y = msg->y_acc;
       position_data.goal_acc_z = msg->z_acc;
       position_data.goal_acc_yaw = msg->yaw_acc;

       position_data.goal_arrival = false;
       position_data.waiting = false;

       position_data.wait_time = msg->wait_time;
       position_data.goal_range = msg->goal_limit;
       position_data.goal_id = msg->goal_id;    
}

void ll_callback(const std_msgs::Float32::ConstPtr& msg)
{
   battery = msg->data;
}

void timerCallback(const ros::TimerEvent&) {
   steady = true;
}

void quickStop(const std_msgs::Bool::ConstPtr& msg) {
   stopped = msg->data;
}

void joy_callback(const sensor_msgs::Joy::ConstPtr& msg)
{
   if(msg->buttons[1] && msg->buttons[4] && msg->buttons[5]) {
	ROS_INFO("Joy Command Motor Kill!");
	stopped = true;
   }
}

float limit(float input, float ceiling, float floor) 
{
   if(input > ceiling) {
      return ceiling;
   }else if(input < floor) {
      return floor;
   }else {
      return input;
   }
}

void update_controller(struct PID_DATA * controller_ptr, struct POS_DATA * position_ptr)
{
   /* ---- P calculation ---- */
   controller_ptr->error_x = (position_ptr->goal_x) - (position_ptr->pos_x);
   controller_ptr->error_y = (position_ptr->goal_y) - (position_ptr->pos_y);
   controller_ptr->error_z = (position_ptr->goal_z) - (position_ptr->pos_z);
   controller_ptr->error_yaw = (position_ptr->goal_yaw) - (position_ptr->pos_yaw);

   //ROS_INFO("X: %f, Y: %f", controller_ptr->error_x, controller_ptr->error_y);

   /* ---- I calculation ---- */
   controller_ptr->integral_x += (controller_ptr->error_x) * dt;
   controller_ptr->integral_y += (controller_ptr->error_y) * dt;
   controller_ptr->integral_z += (controller_ptr->error_z) * dt;
   controller_ptr->integral_yaw += (controller_ptr->error_yaw) * dt;

   /* ---- Vel calculation ---- */
   controller_ptr->error_x_vel = (position_ptr->goal_vel_x) - 
                                 (position_ptr->vel_x);
   controller_ptr->error_y_vel = (position_ptr->goal_vel_y) - 
                                 (position_ptr->vel_y);
   controller_ptr->error_z_vel = (position_ptr->goal_vel_z) - 
                                 (position_ptr->vel_z); 
   controller_ptr->error_yaw_vel = (position_ptr->goal_vel_yaw) - 
                                   (position_ptr->vel_yaw);

   /* ---- Acc calculation ---- */
   controller_ptr->error_x_acc = (position_ptr->goal_acc_x) - 
                                 (position_ptr->acc_x);
   controller_ptr->error_y_acc = (position_ptr->goal_acc_y) - 
                                 (position_ptr->acc_y);
   controller_ptr->error_z_acc = (position_ptr->goal_acc_z) - 
                                 (position_ptr->acc_z); 
   controller_ptr->error_yaw_acc = (position_ptr->goal_acc_yaw) - 
                                   (position_ptr->acc_yaw);

   /* ---- Windup Prevention ---- */
   controller_ptr->integral_x = limit(controller_ptr->integral_x, INTEGRAL_LIMIT, -INTEGRAL_LIMIT);
   controller_ptr->integral_y = limit(controller_ptr->integral_y, INTEGRAL_LIMIT, -INTEGRAL_LIMIT);
   controller_ptr->integral_z = limit(controller_ptr->integral_z, INTEGRAL_LIMIT, -INTEGRAL_LIMIT);
   controller_ptr->integral_yaw = limit(controller_ptr->integral_yaw, INTEGRAL_LIMIT, -INTEGRAL_LIMIT);
}

void update_real_cmd(struct PID_DATA * ctl_ptr, struct POS_DATA * pos_ptr)
{
   double roll;
   double pitch;
   double thrust;
   double yaw;

   yaw = -(k_p_yaw * (ctl_ptr->error_yaw) + 
           k_i_yaw * (ctl_ptr->integral_yaw) + 
           k_d_yaw * (ctl_ptr->error_yaw_vel) +
           k_a_yaw * (ctl_ptr->error_yaw_acc));

   yaw = limit(yaw, YAW_MAX, YAW_MIN);
   
   if(!isnan(yaw)) {
      real_cmd.angular.z = yaw;
   }else {
      //ROS_INFO("Yaw calculation yielded nan");
   }

   pitch = ((k_p_xy * (ctl_ptr->error_x) + 
              k_i_xy * (ctl_ptr->integral_x) + 
              k_d_xy * (ctl_ptr->error_x_vel) + 
	      k_a_xy * (ctl_ptr->error_x_acc)) *
              cos(pos_ptr->pos_yaw)) +

           ((k_p_xy * (ctl_ptr->error_y) + 
              k_i_xy * (ctl_ptr->integral_y) + 
              k_d_xy * (ctl_ptr->error_y_vel) + 
	      k_a_xy * (ctl_ptr->error_y_acc)) *
              sin(pos_ptr->pos_yaw)); 
   
   pitch = limit(pitch, PITCH_MAX, PITCH_MIN);

   if(!isnan(pitch)) {
      real_cmd.linear.x = pitch;
   }else {
      //ROS_INFO("Pitch calculation yielded nan");
   }

   roll = ((k_p_xy * (ctl_ptr->error_x) + 
            k_i_xy * (ctl_ptr->integral_x) + 
            k_d_xy * (ctl_ptr->error_x_vel) +
	    k_a_xy * (ctl_ptr->error_x_acc)) *
            sin(pos_ptr->pos_yaw)) -

          ((k_p_xy * (ctl_ptr->error_y) + 
             k_i_xy * (ctl_ptr->integral_y) + 
             k_d_xy * (ctl_ptr->error_y_vel) + 
	     k_a_xy * (ctl_ptr->error_y_acc)) *
             cos(pos_ptr->pos_yaw));

   roll = limit(roll, ROLL_MAX, ROLL_MIN);

   if(!isnan(roll)) {
      real_cmd.linear.y = roll;
   }else {
      //ROS_INFO("Roll calculation yielded nan");
   }

   thrust = THRUST_SCALE * (k_p_z * (ctl_ptr->error_z) + 
            k_i_z * (ctl_ptr->integral_z) + 
            k_d_z * (ctl_ptr->error_z_vel) +
	    k_a_z * (ctl_ptr->error_z_acc)) + G_TH;

   thrust = limit(thrust, THRUST_MAX, THRUST_MIN);

   if(!isnan(thrust)) {
      real_cmd.linear.z = thrust;
   }else {
      //ROS_INFO("Thrust calculation yielded nan");
   }
}

int main(int argc, char** argv) {
   
   ros::init(argc, argv, "cflie_pos_controller");
   ros::NodeHandle nh;

   dynamic_reconfigure::Server<cflie_controller::tunerConfig> server;
   dynamic_reconfigure::Server<cflie_controller::tunerConfig>::CallbackType f;
   f = boost::bind(&callback, _1, _2);
   server.setCallback(f);

   ROS_INFO("Dynamic Reconfigure Server Started");

   ros::Timer timer_event = nh.createTimer(ros::Duration(1.0), timerCallback, true);
   timer_event.stop();

   string world_frame, name, frame;
   ros::param::get("~world_frame", world_frame);
   ros::param::get("~name", name);
   ros::param::get("~quad_frame", frame);

   ros::param::get("~k_p_xy", k_p_xy); 
   ros::param::get("~k_i_xy", k_i_xy); 
   ros::param::get("~k_d_xy", k_d_xy);
   ros::param::get("~k_a_xy", k_a_xy);

   ros::param::get("~k_p_z", k_p_z); 
   ros::param::get("~k_i_z", k_i_z); 
   ros::param::get("~k_d_z", k_d_z);   
   ros::param::get("~k_a_z", k_a_z);
 
   ros::param::get("~k_p_yaw", k_p_yaw); 
   ros::param::get("~k_i_yaw", k_i_yaw); 
   ros::param::get("~k_d_yaw", k_d_yaw);
   ros::param::get("~k_a_yaw", k_a_yaw);

   accel_quad_cmd = nh.advertise<geometry_msgs::Twist>(name + "/cmd_vel", 10);
   goal_feedback = nh.advertise<pc_asctec_sim::pc_feedback>(name + "/goal_feedback", 10);
   state_pub = nh.advertise<pc_asctec_sim::pc_state>(name + "/state", 10);

   pc_goal_cmd = nh.subscribe(name + "/pos_goals", 1, goal_callback);
   joy_call = nh.subscribe("/joy",10,joy_callback);
   bat_sub = nh.subscribe(name + "/battery", 1, ll_callback);
   stop_sub = nh.subscribe(name + "/stop", 1, quickStop);

   ros::Rate rate(CONTROL_RATE);

   tf::StampedTransform transform;
   tf::TransformListener listener;

   PID_DATA * controller_ptr = &ctl_data;
   POS_DATA * position_ptr = &position_data;

   init(position_ptr, controller_ptr);
   listener.waitForTransform(world_frame, frame, 
                             ros::Time(0), ros::Duration(3.0));
   ros::spinOnce();

   listener.lookupTransform(world_frame, frame, ros::Time(0), transform);
   position_ptr->pos_x = transform.getOrigin().x();
   position_ptr->pos_y = transform.getOrigin().y();
   position_ptr->pos_z = transform.getOrigin().z();  
   position_ptr->pos_yaw = tf::getYaw(transform.getRotation());

   position_ptr->pos_x_past = transform.getOrigin().x();
   position_ptr->pos_y_past = transform.getOrigin().y();
   position_ptr->pos_z_past = transform.getOrigin().z(); 
   position_ptr->pos_yaw_past = tf::getYaw(transform.getRotation());

   position_ptr->goal_x = transform.getOrigin().x();
   position_ptr->goal_y = transform.getOrigin().y();
   
   now = transform.stamp_;
   past = now;

   double timer_past, timer = 0;
   bool check = false;
   ROS_INFO("Controller active!!");

   while (ros::ok()) {
    //Check Battery Life
      check_bat();
    
    //Grab new transform data
      listener.lookupTransform(world_frame, frame, ros::Time(0), transform);
      now = transform.stamp_;
      dt = (now.toSec() - past.toSec()) + (now.toNSec() - past.toNSec())/pow(10,10);

    //Set t-1 values
      position_ptr->pos_x_past = position_ptr->pos_x;
      position_ptr->pos_y_past = position_ptr->pos_y;
      position_ptr->pos_z_past = position_ptr->pos_z;
      position_ptr->pos_yaw_past = position_ptr->pos_yaw;

      position_ptr->vel_x_past = position_ptr->vel_x;
      position_ptr->vel_y_past = position_ptr->vel_y;
      position_ptr->vel_z_past = position_ptr->vel_z;
      position_ptr->vel_yaw_past = position_ptr->vel_yaw;
      
      past = now;

    //Get new position values
      position_ptr->pos_x = transform.getOrigin().x();
      position_ptr->pos_y = transform.getOrigin().y();
      position_ptr->pos_z = transform.getOrigin().z();  
      position_ptr->pos_yaw = tf::getYaw(transform.getRotation()) + 2*M_PI*yaw_counter;

    //Adjust yaw counter
      float yaw_dif = position_ptr->pos_yaw - position_ptr->pos_yaw_past;
      if(abs(yaw_dif) > M_PI) {
	 if(!check) {
            if(yaw_dif < 0.0) {
	       yaw_counter += 1;
	    }else {
	       yaw_counter -= 1;
	    }
	    check = true;
	 }else {
	    check = false;
	 }
      }

    //Calculate velocity values
      position_ptr->vel_x = ((position_ptr->pos_x) - (position_ptr->pos_x_past)) / dt;
      position_ptr->vel_y = ((position_ptr->pos_y) - (position_ptr->pos_y_past)) / dt;
      position_ptr->vel_z = ((position_ptr->pos_z) - (position_ptr->pos_z_past)) / dt;
      position_ptr->vel_yaw = ((position_ptr->pos_yaw) - (position_ptr->pos_yaw_past)) / dt;

    //Calculate accel values
      position_ptr->acc_x = ((position_ptr->vel_x) - (position_ptr->vel_x_past)) / dt;
      position_ptr->acc_y = ((position_ptr->vel_y) - (position_ptr->vel_y_past)) / dt;
      position_ptr->acc_z = ((position_ptr->vel_z) - (position_ptr->vel_z_past)) / dt;
      position_ptr->acc_yaw = ((position_ptr->vel_yaw) - (position_ptr->vel_yaw_past)) / dt;

    //Update controller values
      update_controller(controller_ptr, position_ptr);  

    //Check if target goal was accomplished -> update new goal and publish goal confirm  
      goal_arrived(controller_ptr, position_ptr);

      if(!stopped) {
      update_real_cmd(controller_ptr, position_ptr);
      accel_quad_cmd.publish(real_cmd);

      }else {
	 position_ptr->goal_x = position_ptr->pos_x;
	 position_ptr->goal_y = position_ptr->pos_y;
	 position_ptr->goal_z = position_ptr->pos_z;
	 position_ptr->goal_yaw = position_ptr->pos_yaw;

	 position_ptr->goal_vel_x = 0.0;
	 position_ptr->goal_vel_y = 0.0;
	 position_ptr->goal_vel_z = 0.0;
	 position_ptr->goal_vel_yaw = 0.0;

	 position_ptr->goal_acc_x = 0.0;
	 position_ptr->goal_acc_y = 0.0;
	 position_ptr->goal_acc_z = 0.0;
	 position_ptr->goal_acc_yaw =0.0;

	 real_cmd.linear.x = 0.0;
	 real_cmd.linear.y = 0.0;
	 real_cmd.linear.z = 0.0;
	 real_cmd.angular.z = 0.0;
         accel_quad_cmd.publish(real_cmd);
      }

    //Fill State Data and Publish
      state_data.event = now;

      state_data.x = position_ptr->pos_x;
      state_data.x_vel = position_ptr->vel_x;
      state_data.x_acc = position_ptr->acc_x;
      state_data.x_goal = position_ptr->goal_x;
      state_data.x_vel_goal = position_ptr->goal_vel_x;
      state_data.x_acc_goal = position_ptr->goal_acc_x;

      state_data.y = position_ptr->pos_y;
      state_data.y_vel = position_ptr->vel_y;
      state_data.y_acc = position_ptr->acc_y;
      state_data.y_goal = position_ptr->goal_y;
      state_data.y_vel_goal = position_ptr->goal_vel_y;
      state_data.y_acc_goal = position_ptr->goal_acc_y;

      state_data.z = position_ptr->pos_z;
      state_data.z_vel = position_ptr->vel_z;
      state_data.z_acc = position_ptr->acc_z;
      state_data.z_goal = position_ptr->goal_z;
      state_data.z_vel_goal = position_ptr->goal_vel_z;
      state_data.z_acc_goal = position_ptr->goal_acc_z;

      state_data.yaw = position_ptr->pos_yaw;
      state_data.yaw_vel = position_ptr->vel_yaw;
      state_data.yaw_acc = position_ptr->acc_yaw;
      state_data.yaw_goal = position_ptr->goal_yaw;
      state_data.yaw_vel_goal = position_ptr->goal_vel_yaw;
      state_data.yaw_acc_goal = position_ptr->goal_acc_yaw;

      state_pub.publish(state_data);

      ros::spinOnce();
      rate.sleep();
   }
   return 0;
}
