#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <tf/tf.h>
#include <eigen3/Eigen/Dense>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <sstream>
#include <iostream>
#include <fstream>
#include <chrono>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>


// Setup Defines and Macros
#define FREQ 30.0f
#define dt   1.0f/FREQ // time of 1 loop
#define SHOW_JOINT

// Setup Global Variables
nav_msgs::Odometry stm_gps;
nav_msgs::Odometry px4_gps;
geometry_msgs::Twist ekf_pos;
geometry_msgs::Twist mpc_cmd;
sensor_msgs::Imu stm_imu;
sensor_msgs::Imu px4_imu;
sensor_msgs::JointState joint_state;
double stm_theta, stm_theta_dot, stm_delta, stm_velocity, stm_acceleration, stm_phi_dot, stm_gps_var, px4_gps_varx, px4_gps_vary;
bool POS_FLAG = false;

// Function Forward Declaration
void WsgToTWD(double raw_lat, double raw_lon, float* output);

// Setup Callback Functions
void ekfCallback(const geometry_msgs::Twist::ConstPtr& msg){  
  /* msg->linear.x:  pos x
   * msg->linear.y:  pos y
   * msg->angular.z: phi angle
   * msg->linear.z:  velocity */
  ekf_pos = *msg;
}

void stmStateCallback(const geometry_msgs::Twist::ConstPtr& msg){
  stm_theta = msg->linear.x;
  stm_theta_dot = msg->linear.y;
  stm_delta = msg->linear.z;
  stm_velocity = msg->angular.x;
  stm_acceleration = msg->angular.y;
  stm_phi_dot = msg->angular.z;
}

void stmGPSCallback(const geometry_msgs::Twist::ConstPtr& msg){
  float pos[2] = {0};
  double lat = msg->linear.x;
  double lon = msg->linear.y;
  stm_gps_var = msg->linear.z;
  WsgToTWD(lat, lon, pos);
  stm_gps.pose.pose.position.x = pos[0];
  stm_gps.pose.pose.position.y = pos[1];
}

//void stmIMUCallback(const geometry_msgs::Twist::ConstPtr& msg){}

void px4IMUCallback(const sensor_msgs::Imu::ConstPtr& msg){
  px4_imu = *msg;
  px4_imu.header.stamp = ros::Time::now();
  px4_imu.header.frame_id = "/bike/base_link";
  px4_gps.pose.pose.orientation = msg->orientation;
}

void px4PosCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
  float pos[2] = {0};
  double lat = msg->latitude;
  double lon = msg->longitude;
  WsgToTWD(lat, lon, pos);
  px4_gps.pose.pose.position.x = pos[0];
  px4_gps.pose.pose.position.y = pos[1];
  POS_FLAG = true;
  px4_gps_varx = msg->position_covariance[0];
  px4_gps_vary = msg->position_covariance[4];
}

void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg){
  mpc_cmd = *msg;
}

int main(int argc, char** argv){
  // Setup ros node
  ros::init(argc, argv, "bike_viz_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  ros::Rate loop_rate(FREQ); 

  // Define parameters
  geometry_msgs::PoseStamped gps_pose_stamp;
  nav_msgs::Path gps_path;

  geometry_msgs::PoseStamped gps_stm_pose_stamp;
  nav_msgs::Path gps_stm_path;

  geometry_msgs::PoseStamped ekf_pose_stamp;
  nav_msgs::Path ekf_path;
  // float init_pos[2] = {76, -6};
  // float init_pos[2] = {75.5, -8};
  float INIT_X, INIT_Y, STM_GPS, PUB_EKF, USE_POS_BIAS, PUB_GPS, RECORD_DATA;
  private_nh.getParam("INIT_X",INIT_X);
  private_nh.getParam("INIT_Y",INIT_Y);
  private_nh.getParam("STM_GPS",STM_GPS);
  private_nh.getParam("USE_POS_BIAS",USE_POS_BIAS);
  private_nh.getParam("PUB_GPS",PUB_GPS);
  private_nh.getParam("PUB_EKF",PUB_EKF);
  private_nh.getParam("RECORD_DATA",RECORD_DATA);
  float init_pos[2] = {INIT_X, INIT_Y};
  bool V_FLAG = false;
  float bias_x = 0, bias_y = 0, bias_counter = 0, bias_st_x = 0, bias_st_y = 0;

  std::fstream EKF_log;
  static double simtim = 0;

  // Setup subscribers
  ros::Subscriber stm_state_sub = nh.subscribe<geometry_msgs::Twist>("/output", 10, &stmStateCallback);
  ros::Subscriber stm_gps_sub = nh.subscribe<geometry_msgs::Twist>("/stm_gps", 10, &stmGPSCallback);
  ros::Subscriber ekf_sub = nh.subscribe<geometry_msgs::Twist>("bike/ekf_pos", 10, &ekfCallback);
  ros::Subscriber px4_imu_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, &px4IMUCallback);
  ros::Subscriber px4_pos_sub = nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 10, &px4PosCallback);
  ros::Subscriber cmd_sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &cmdCallback);
  
  // Setup publishers
  ros::Publisher gps_path_pub = nh.advertise<nav_msgs::Path>("/bike/gps_trajectory",10);
  ros::Publisher gps_stm_path_pub = nh.advertise<nav_msgs::Path>("/bike/gps_stm_trajectory",10);
  ros::Publisher ekf_path_pub = nh.advertise<nav_msgs::Path>("/bike/ekf_trajectory",10);
  #ifdef SHOW_JOINT
  ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("/bike/joint_states",10);
  #endif

  // Setup tf2 broadcaters
  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped static_transformStamped;


  while (ros::ok())
  { 
    if(!POS_FLAG){
      ROS_INFO("Waiting for topic ...");
      // Publish /bike/odom tf respect to the /map
      static_transformStamped.header.stamp = ros::Time::now();
      static_transformStamped.header.frame_id = "/map";
      static_transformStamped.child_frame_id = "/bike/odom";
      static_transformStamped.transform.translation.x = init_pos[0];
      static_transformStamped.transform.translation.y = init_pos[1];
      static_transformStamped.transform.translation.z = 0.0;
      static_transformStamped.transform.rotation.x = 0.0;
      static_transformStamped.transform.rotation.y = 0.0;
      static_transformStamped.transform.rotation.z = 1.0;
      static_transformStamped.transform.rotation.w = 0.0;
      static_broadcaster.sendTransform(static_transformStamped);
    }else{
      #ifdef SHOW_JOINT
        // Publish the joint states of the bicycle
      joint_state.header.stamp = ros::Time::now();
      joint_state.name.resize(3);
      joint_state.position.resize(3);
      joint_state.name[0] = "Rev2";     // front fork
      joint_state.name[1] = "Rev1";     // front wheel
      joint_state.name[2] = "Rev3";     // back wheel
      joint_state.position[0] = -stm_delta; // (-) because the mx motor is oppositely installed on the actual system 
      joint_state.position[1] = stm_velocity;
      joint_state.position[2] = stm_velocity;
      joint_state_pub.publish(joint_state);
      #endif

      if(!V_FLAG){
        if(USE_POS_BIAS){
          bias_counter++;
          bias_x = (bias_x*(bias_counter-1) + init_pos[0] - ekf_pos.linear.x)/bias_counter;
          bias_y = (bias_y*(bias_counter-1) + init_pos[1] - ekf_pos.linear.y)/bias_counter;
          ROS_INFO("PX4 POS BIAS: %.3f %.3f", bias_x, bias_y);
          
          if(STM_GPS){
            bias_st_x = (bias_st_x*(bias_counter-1) + init_pos[0] - stm_gps.pose.pose.position.x)/bias_counter;
            bias_st_y = (bias_st_y*(bias_counter-1) + init_pos[1] - stm_gps.pose.pose.position.y)/bias_counter;
            ROS_INFO("STM POS BIAS: %.3f %.3f | %.3f %.3f", bias_st_x, bias_st_y, stm_gps.pose.pose.position.x, stm_gps.pose.pose.position.y);
          }
        }
       

        if(!USE_POS_BIAS){
          bias_x = 0;
          bias_y = 0;
          bias_st_x = 0;
          bias_st_y = 0;
        }

        if(stm_velocity > 1.0){
          V_FLAG = true;
        }
      }else{
        // Publish /bike/odom tf respect to the /map
        static_transformStamped.header.stamp = ros::Time::now();
        static_transformStamped.header.frame_id = "/map";
        static_transformStamped.child_frame_id = "/bike/odom";
        static_transformStamped.transform.translation.x = ekf_pos.linear.x + bias_x;
        static_transformStamped.transform.translation.y = ekf_pos.linear.y + bias_y;
        static_transformStamped.transform.translation.z = 0.0;

        tf2::Quaternion EKFQuaternion;
        
        #ifdef SHOW_JOINT
        double px4_roll, px4_pitch, px4_yaw;
        tf::Quaternion q(px4_imu.orientation.x, px4_imu.orientation.y, px4_imu.orientation.z, px4_imu.orientation.w);
        tf::Matrix3x3 m(q);
        m.getRPY(px4_roll, px4_pitch, px4_yaw);
        
        // EKFQuaternion.setRPY(px4_roll,px4_pitch,ekf_pos.angular.z);
        EKFQuaternion.setRPY(0,0,ekf_pos.angular.z);
        EKFQuaternion=EKFQuaternion.normalize();
        static_transformStamped.transform.rotation.x = EKFQuaternion.getX();
        static_transformStamped.transform.rotation.y = EKFQuaternion.getY();
        static_transformStamped.transform.rotation.z = EKFQuaternion.getZ();
        static_transformStamped.transform.rotation.w = EKFQuaternion.getW();
        #endif

        

        // static_transformStamped.transform.rotation.x = px4_imu.orientation.x;
        // static_transformStamped.transform.rotation.y = px4_imu.orientation.y;
        // static_transformStamped.transform.rotation.z = px4_imu.orientation.z;
        // static_transformStamped.transform.rotation.w = px4_imu.orientation.w;
        static_broadcaster.sendTransform(static_transformStamped);

        

        if(PUB_GPS){ // This data should be published through the EKF node
          // Publish the recieved GPS trajectory 
          gps_pose_stamp.pose.position.x = px4_gps.pose.pose.position.x + bias_x;
          gps_pose_stamp.pose.position.y = px4_gps.pose.pose.position.y + bias_y;
          gps_pose_stamp.pose.position.z = 0;
          gps_pose_stamp.header.stamp=ros::Time::now();
          gps_path.header.frame_id = "/map";
          gps_path.poses.push_back(gps_pose_stamp);
          gps_path_pub.publish(gps_path); 

          if(STM_GPS){
            gps_stm_pose_stamp.pose.position.x = stm_gps.pose.pose.position.x + bias_st_x;
            gps_stm_pose_stamp.pose.position.y = stm_gps.pose.pose.position.y + bias_st_y;
            gps_stm_pose_stamp.pose.position.z = 0;
            gps_stm_pose_stamp.header.stamp=ros::Time::now();
            gps_stm_path.header.frame_id = "/map";
            if(std::isfinite(gps_stm_pose_stamp.pose.position.x)&&std::isfinite(gps_stm_pose_stamp.pose.position.y)){
              gps_stm_path.poses.push_back(gps_stm_pose_stamp);
            }

            gps_stm_path_pub.publish(gps_stm_path); 
            
          }
        }

        if(PUB_EKF){
          // Publish the recieved EKF trajectory 
          ekf_pose_stamp.pose.position.x = ekf_pos.linear.x + bias_x;
          ekf_pose_stamp.pose.position.y = ekf_pos.linear.y + bias_y;
          ekf_pose_stamp.pose.position.z = 0;
          ekf_pose_stamp.header.stamp=ros::Time::now();
          ekf_path.header.frame_id = "/map";
          ekf_path.poses.push_back(ekf_pose_stamp);
          ekf_path_pub.publish(ekf_path); 
        }
      }
    }

    
    if(RECORD_DATA){
      /* Save results to a .csv file 
       * Simulation time
       * EKF x y phi
       * Pixhawk GPS x y
       * STM32 RTK GPS x y
       * STM32 theta theta_dot delta v0 a0 phi_dot
       * px4 theta theta_dot a0 phi_dot
       * px4 varx vary 
       * stm32 rtk var
       * vd wd
       * */
      double px4_roll, px4_pitch, px4_yaw;
      tf::Quaternion q(px4_imu.orientation.x, px4_imu.orientation.y, px4_imu.orientation.z, px4_imu.orientation.w);
      tf::Matrix3x3 m(q);
      m.getRPY(px4_roll, px4_pitch, px4_yaw);

      EKF_log.open("Experiment_log1.csv", std::ios::out | std::ios::app);
      EKF_log << simtim << " " << (float)ekf_pose_stamp.pose.position.x<< " " << ekf_pose_stamp.pose.position.y << " " << ekf_pos.angular.z 
      << " " << gps_pose_stamp.pose.position.x << " " << gps_pose_stamp.pose.position.y
      << " " << gps_stm_pose_stamp.pose.position.x << " " << gps_stm_pose_stamp.pose.position.y 
      << " " << stm_theta<<" "<<stm_theta_dot<<" "<<stm_delta<<" "<<stm_velocity<<" "<<stm_acceleration << " "<<stm_phi_dot
      << " " << px4_roll << " " <<px4_imu.angular_velocity.x<<" "<<px4_imu.linear_acceleration.x << " " << px4_yaw
      << " " << px4_gps_varx<< " " << px4_gps_vary <<" " << stm_gps_var
      << " " << (float)mpc_cmd.linear.x <<" "<< (float)mpc_cmd.linear.y <<std::endl;
      EKF_log.close();
      simtim += dt;
    }  
    
    ros::spinOnce();
    loop_rate.sleep();
  }
};

// Transform coordinates from WGS84 (lon/lat) to TWD97 representation
void WsgToTWD(double raw_lat, double raw_lon, float* output){
  float DEG2RAD_D = 0.0174532925199432957;
  float x_axis_range = 250;		  // where I think the map is on -250~250 square
  float y_axis_range = 250;		  
  double gps_driftx = 249472.0;	// to adjust gps x,y coordinate to near the original point
  double gps_drifty = 2743027.0;
  // reference: http://sask989.blogspot.com/2012/05/wgs84totwd97.html
  double a = 6378137.0;
  double b = 6356752.3142451;
  double lon0 = 121 * DEG2RAD_D;
  double k0 = 0.9999;
  double dx = 250000;
  double dy = 0;
  double lat = raw_lat*DEG2RAD_D;
  double lon = raw_lon*DEG2RAD_D;

  double e = (1 - pow(b,2) / pow(a,2));
  double e2 = (1 - pow(b, 2) / pow(a, 2)) / (pow(b, 2) / pow(a, 2));

  double V = a / sqrt(1 - e * pow(sin(lat), 2));
  double T = pow(tan(lat), 2);
  double C = e2 * pow(cos(lat), 2);
  double A = cos(lat) * (lon - lon0);
  double M = a * ((1.0 - e / 4.0 - 3.0 * pow(e, 2) / 64.0 - 5.0 * pow(e, 3) / 256.0) * lat
			- (3.0 * e / 8.0 + 3.0 * pow(e, 2) / 32.0 + 45.0 * pow(e, 3) / 1024.0) * sin(2.0 * lat)
			+ (15.0 * pow(e, 2) / 256.0 + 45.0 * pow(e, 3) / 1024.0) * sin(4.0 * lat)
			- (35.0 * pow(e, 3) / 3072.0) * sin(6.0 * lat));
			       
  double x = dx + k0 * V * (A + (1 - T + C) * pow(A, 3) / 6 + (5 - 18 * T + pow(T, 2) + 72 * C - 58 * e2) * pow(A, 5) / 120);
  double y = dy + k0 * (M + V * tan(lat) * (pow(A, 2) / 2 + (5 - T + 9 * C + 4 * pow(C, 2)) * pow(A, 4) / 24 + ( 61 - 58 * T + pow(T, 2) + 600 * C - 330 * e2) * pow(A, 6) / 720));

  output[0] = x-gps_driftx;
  output[1] = y-gps_drifty;
}
