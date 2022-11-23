#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <fstream>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>

#define T_HRZ 2.5f			    // Prediction time horizon
#define F_STP 20.0f			    // Frequency of prediction (control freq)
#define T_STP 1/F_STP			  // Period of prediction
#define N_HRZ T_HRZ*F_STP		// N steps in a time horizon
#define ST_FREQ 100
#define WHEEL_R 0.325f
#define SPEEDUP_P 100
#define CTRL_V 2

// Self-define structures
struct Control {
	float vd, wd;
};
struct State {
  float x, y, phi, v0;
};
struct Track {
  // Track fcn in the form of by + ax + c =0
  float b, a, c;
};

// Initialize functions
void kernelmain(Control* output_cmd, Control* host_U, State* output_state, int freq);
void trackInit();
void randomInit(int seed);
void quaternion2RPY(sensor_msgs::Imu input, double *output);

// Global variables
nav_msgs::Odometry pos;

Control output_cmd;

// Other params for callback fcn
sensor_msgs::Imu imu_data;
geometry_msgs::Twist command;
sensor_msgs::JointState joint_state;

// Setup callback function
void odomPosCallback(const nav_msgs::Odometry::ConstPtr& msg){
  pos = *msg;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
  imu_data = *msg;
}

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg){
  // 0: front-wheel, 1: front-fork, 2: back-wheel
  // msg->position[1] = delta angle (steering angle response)
  joint_state = *msg;
}

int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "main");
  ros::NodeHandle n;
  ros::Rate loop_rate(int(F_STP));
  
  // Setup variables
  geometry_msgs::Twist cmd;
  geometry_msgs::PoseStamped pose_stamp;
  nav_msgs::Path path;
  geometry_msgs::PoseStamped mpc_pose_stamp;
  nav_msgs::Path mpc_path;
  double v_no_control = 0;
  double vehicle_output[3] = {0};
  State start_pos = {0};
  
  // Setup publishers
  ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  ros::Publisher path_pub = n.advertise<nav_msgs::Path>("/bike/trajectory",10);
  
  // Setup subscribers
  ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("/bike/odom", 10, &odomPosCallback);
  ros::Subscriber joint_sub = n.subscribe<sensor_msgs::JointState>("/bike/joint_states", 10, &jointStateCallback);
  ros::Subscriber imu_sub = n.subscribe<sensor_msgs::Imu>("/bike/imu", 10, &imuCallback);
  

  // Other variables
  int counter = 0;
  double counter_control = 0;
  static double simtim = 0;

  // Output .csv files
  std::fstream MPC_log;

  while (ros::ok())
  {
    // Raise velocity to speed limit before starting MPC control
    if(counter < SPEEDUP_P){
      cmd.linear.x = v_no_control;
      cmd.angular.z = 0;
      cmd_pub.publish(cmd);
      if(v_no_control<CTRL_V){
        v_no_control += double(CTRL_V)/double(SPEEDUP_P);
      }
    }
    else{
      // Generate the command
      if(counter_control<2){
        output_cmd.vd = 2;
        output_cmd.wd = 0;
      }
      if(counter_control>2 && counter_control<4){
        output_cmd.vd = 2;
        output_cmd.wd = 0.5;
      }
      if(counter_control>4 && counter_control<6){
        output_cmd.vd += 0.01;
        output_cmd.wd -= 0.005;
      }
      if(counter_control>6){
        output_cmd.vd = 0.0;
        output_cmd.wd = 0.0;
      }
      
      quaternion2RPY(imu_data, vehicle_output);
      
      // Publish optimal command calculated by MPC
      cmd.linear.x = output_cmd.vd;
      cmd.angular.z = output_cmd.wd;
      cmd_pub.publish(cmd);
      
      // Store ground truth position to trajectory
      pose_stamp.pose.position.x = pos.pose.pose.position.x;
      pose_stamp.pose.position.y = pos.pose.pose.position.y;
      pose_stamp.pose.position.z = 0;
      pose_stamp.header.stamp=ros::Time::now();
      path.header.frame_id = "/map";
      path.poses.push_back(pose_stamp);
    
      // Publish topics
      path_pub.publish(path);
      
      // create a name for the file output [t, x, y, phi, v0, vd, wd, theta, delta]
      MPC_log.open("controller_exp_log.csv", std::ios::out | std::ios::app);
      MPC_log << simtim << " " << pos.pose.pose.position.x<< " " << pos.pose.pose.position.y << " " 
      << vehicle_output[2] << " " << (double)joint_state.velocity[2] * WHEEL_R << " " <<cmd.linear.x  << " " 
      << cmd.angular.z << " " <<vehicle_output[0]<< " "<<(double)joint_state.position[1]<< std::endl;
      MPC_log.close();
      
      counter_control += 0.01;
    }
    counter ++;

    
    simtim += T_STP;
    
    ros::spinOnce();
    loop_rate.sleep();
    
  }
  
  return 0;
}

void quaternion2RPY(sensor_msgs::Imu input, double *output){
  // roll (x-axis rotation)
  double sinr_cosp = 2 * (input.orientation.w * input.orientation.x + input.orientation.y * input.orientation.z);
  double cosr_cosp = 1 - 2 * (input.orientation.x * input.orientation.x + input.orientation.y * input.orientation.y);
  output[0] = std::atan2(sinr_cosp, cosr_cosp);
  
  // pitch (y-axis rotation)
  double sinp = 2 * (input.orientation.w * input.orientation.y - input.orientation.z * input.orientation.x);
  if (std::abs(sinp) >= 1)
      output[1] = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
      output[1] = std::asin(sinp);
      
  // yaw (z-axis rotation)
  double siny_cosp = 2 * (input.orientation.w * input.orientation.z + input.orientation.x * input.orientation.y);
  double cosy_cosp = 1 - 2 * (input.orientation.y * input.orientation.y + input.orientation.z * input.orientation.z);
  output[2] = std::atan2(siny_cosp, cosy_cosp);
}
