#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <tf/tf.h>
#include <eigen3/Eigen/Dense>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <fstream>
#include <chrono>

using namespace Eigen;

//#define ORIG_LON 1.0f      // define longitude origin 
//#define ORIG_LAT 1.0f      // define latitude origin
#define FREQ     30.0f    // loop_rate
#define dt       1.0f/FREQ // time of 1 loop
#define DEG2RAD_D 0.0174532925199432957
#define STM_DATA

geometry_msgs::Twist WsgToTWD(double raw_lat, double raw_lon);

// Global variables
sensor_msgs::NavSatFix gps_data; 
geometry_msgs::Twist gps_pos;
sensor_msgs::NavSatFix gps_raw_data; 
geometry_msgs::Twist gps_raw_pos;
#ifdef STM_DATA
geometry_msgs::Twist gps_stm_pos;
#endif
sensor_msgs::Imu imu_data;
geometry_msgs::Twist ekf_pos;

// Setup callback functions 
void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
  gps_data = *msg;
  gps_pos = WsgToTWD(gps_data.latitude, gps_data.longitude);
}

void gpsRawCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
  gps_raw_data = *msg;
  gps_raw_pos = WsgToTWD(gps_raw_data.latitude, gps_raw_data.longitude);
}

#ifdef STM_DATA
void gpsStmCallback(const geometry_msgs::Twist::ConstPtr& msg){
  gps_stm_pos = WsgToTWD(msg->linear.x, msg->linear.y);
}
#endif

// Get IMU data from the PX4 FCU
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
  imu_data = *msg;
}
  
int main(int argc, char** argv){
  // Setup ros node
  ros::init(argc, argv, "px4_viz_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  ros::Rate loop_rate(FREQ); 
  
  geometry_msgs::PoseStamped gps_pose_stamp;
  geometry_msgs::PoseStamped gps_raw_pose_stamp;
#ifdef STM_DATA
  geometry_msgs::PoseStamped gps_stm_pose_stamp;
#endif
  geometry_msgs::PoseStamped ekf_pose_stamp;
  nav_msgs::Path gps_path;
  nav_msgs::Path gps_raw_path;
#ifdef STM_DATA
  nav_msgs::Path gps_stm_path;
#endif
 
  // Define the subscriber to odometry
  ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, &imuCallback);
  ros::Subscriber gps_sub = nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 10, &gpsCallback);
  ros::Subscriber gps_raw_sub = nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/raw/fix", 10, &gpsRawCallback);
#ifdef STM_DATA
  ros::Subscriber gps_stm_sub = nh.subscribe<geometry_msgs::Twist>("/stm_gps", 10, &gpsStmCallback);
#endif
  
  // Define the publishers for bicycle conroller
  ros::Publisher gps_path_pub = nh.advertise<nav_msgs::Path>("/bike/gps_trajectory",10);
  ros::Publisher gps_raw_path_pub = nh.advertise<nav_msgs::Path>("/bike/gps_raw_trajectory",10);
#ifdef STM_DATA
  ros::Publisher gps_stm_path_pub = nh.advertise<nav_msgs::Path>("/bike/gps_stm_trajectory",10);
#endif
  
  std::fstream pos_log;
  static double simtim = 0;

  // Execute ekf filter and publish the fused state
  while (ros::ok())
  {
    // Publish trajectories
    gps_pose_stamp.pose.position.x = gps_pos.linear.x;
    gps_pose_stamp.pose.position.y = gps_pos.linear.y;
    gps_pose_stamp.pose.position.z = 0;
    gps_pose_stamp.header.stamp=ros::Time::now();
    gps_path.header.frame_id = "/map";
    gps_path.poses.push_back(gps_pose_stamp);
    
    gps_raw_pose_stamp.pose.position.x = gps_raw_pos.linear.x;
    gps_raw_pose_stamp.pose.position.y = gps_raw_pos.linear.y;
    gps_raw_pose_stamp.pose.position.z = 0;
    gps_raw_pose_stamp.header.stamp=ros::Time::now();
    gps_raw_path.header.frame_id = "/map";
    gps_raw_path.poses.push_back(gps_raw_pose_stamp);

#ifdef STM_DATA
    gps_stm_pose_stamp.pose.position.x = gps_stm_pos.linear.x;
    gps_stm_pose_stamp.pose.position.y = gps_stm_pos.linear.y;
    gps_stm_pose_stamp.pose.position.z = 0;
    gps_stm_pose_stamp.header.stamp=ros::Time::now();
    gps_stm_path.header.frame_id = "/map";
    gps_stm_path.poses.push_back(gps_stm_pose_stamp);
#endif
    
    gps_path_pub.publish(gps_path);    // Publish GPS trajectory
    gps_raw_path_pub.publish(gps_raw_path);	// Publish GPS raw trajectory

#ifdef STM_DATA
    gps_stm_path_pub.publish(gps_stm_path);
#endif
    
#ifndef STM_DATA
    pos_log.open("EKF_log.csv", std::ios::out | std::ios::app);
    pos_log << simtim << " " << gps_raw_pose_stamp.pose.position.x<< " " << gps_raw_pose_stamp.pose.position.y << " " 
    << gps_pose_stamp.pose.position.x << " " << gps_pose_stamp.pose.position.y << std::endl;
#endif
#ifdef STM_DATA
    pos_log.open("EKF_log.csv", std::ios::out | std::ios::app);
    pos_log << simtim << " " << gps_raw_pose_stamp.pose.position.x << " " << gps_raw_pose_stamp.pose.position.y << " " 
    << gps_pose_stamp.pose.position.x << " " << gps_pose_stamp.pose.position.y << " " << gps_stm_pose_stamp.pose.position.x 
    << " " << gps_stm_pose_stamp.pose.position.y << std::endl;
#endif

    // ROS_INFO("HI!!");
    pos_log.close();
    simtim += dt;
    ros::spinOnce();
    loop_rate.sleep();
  }
};

// Transform coordinates from WGS84 (lon/lat) to TWD97 representation
geometry_msgs::Twist WsgToTWD(double raw_lat, double raw_lon){
  geometry_msgs::Twist output;
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

  output.linear.x = x-gps_driftx;
  output.linear.y = y-gps_drifty;
  return output;
}


//rostopic pub -r 50 /bicycle_data geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
//roslaunch mavros px4.launch fcu_url:=/dev/ttyUSB0:921600
//rosbag record -O gps_rec.bag /mavros/global_position/raw/fix /mavros/global_position/global /mavros/imu/data

