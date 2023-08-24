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

void extended_kalman_filter(sensor_msgs::NavSatFix gps_data, geometry_msgs::Twist gps_pos, sensor_msgs::Imu imu_data, geometry_msgs::Twist bicycle_data, geometry_msgs::Twist* state);

// Global variables
sensor_msgs::NavSatFix gps_data; 
geometry_msgs::Twist gps_pos;
sensor_msgs::Imu imu_data;
geometry_msgs::Twist ekf_pos;
geometry_msgs::Twist bicycle_data;

// Global flags for separate subscribers
volatile bool PX4_FLAG = 0;
volatile bool BIKE_FLAG = 0;
bool START_EKF = 0;

// Setup callback functions 
// Transform coordinates from WGS84 (lon/lat) to TWD97 representation
void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
  gps_data = *msg;
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
  double lat = gps_data.latitude*DEG2RAD_D;
  double lon = gps_data.longitude*DEG2RAD_D;

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

  gps_pos.linear.x = x-gps_driftx;
  gps_pos.linear.y = y-gps_drifty;
  
  PX4_FLAG = 1;
}


// Get IMU data from the PX4 FCU
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
  imu_data = *msg;
}

// Get bicycle data from stm32 through rosserial
void bicycleCallback(const geometry_msgs::Twist::ConstPtr& msg){
  bicycle_data = *msg;
  BIKE_FLAG = 1;
}
  
int main(int argc, char** argv){
  // Setup ros node
  ros::init(argc, argv, "px4_ekf_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  ros::Rate loop_rate(FREQ); 
  
  geometry_msgs::PoseStamped gps_pose_stamp;
  geometry_msgs::PoseStamped ekf_pose_stamp;
  nav_msgs::Path gps_path;
  nav_msgs::Path ekf_path;
 
  // Define the subscriber to odometry
  ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, &imuCallback);
  ros::Subscriber gps_sub = nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 10, &gpsCallback);
  //ros::Subscriber gps_sub = nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/raw/fix", 10, &gpsCallback);
  ros::Subscriber bicycle_sub = nh.subscribe<geometry_msgs::Twist>("/output", 10, &bicycleCallback);
  
  // Define the publishers for bicycle conroller
  ros::Publisher ekf_pub = nh.advertise<geometry_msgs::Twist>("bike/ekf_pos", 10);
  
  ros::Publisher gps_path_pub = nh.advertise<nav_msgs::Path>("/bike/gps_trajectory",10);
  ros::Publisher ekf_path_pub = nh.advertise<nav_msgs::Path>("/bike/ekf_trajectory",10);
  
  // Setup variables
  geometry_msgs::Twist state;
  state.linear.x = 79.5;
  state.linear.y = -3.0;
  state.angular.z = 2.7;
  state.linear.z = 0.0;
  //123.795  55.916
//  state.linear.x = 130.0;
//  state.linear.y = 57.0;
//  state.angular.z = 2.7;
//  state.linear.z = 0.0;
  long int counter = 0;

  std::fstream EKF_log;
  static double simtim = 0;
  
  while(ros::ok() && !START_EKF){
    if(BIKE_FLAG && PX4_FLAG){
    START_EKF = 1;
    ROS_INFO("== STM32 rosserial data recieved! ==");
    ROS_INFO("==      PX4 FCU data recieved!    ==");
    }
    counter ++;
    if(counter > 100){
      if(!PX4_FLAG){ROS_INFO("[ERROR]: No PX4 FCU data");}
      if(!BIKE_FLAG){ROS_INFO("[ERROR]: No STM32 rosserial data");}
      counter = 0;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  if(START_EKF){ROS_INFO("==            EKF START!!         ==");}
  
  // Execute ekf filter and publish the fused state
  while (ros::ok())
  {
    if(START_EKF){
    	extended_kalman_filter(gps_data, gps_pos, imu_data, bicycle_data, &state);
    }
    // Publish trajectories
    gps_pose_stamp.pose.position.x = gps_pos.linear.x;
    gps_pose_stamp.pose.position.y = gps_pos.linear.y;
    gps_pose_stamp.pose.position.z = 0;
    gps_pose_stamp.header.stamp=ros::Time::now();
    gps_path.header.frame_id = "/map";
    gps_path.poses.push_back(gps_pose_stamp);
    
    ekf_pose_stamp.pose.position.x = state.linear.x;
    ekf_pose_stamp.pose.position.y = state.linear.y;
    ekf_pose_stamp.pose.position.z = 0;
    ekf_pose_stamp.header.stamp=ros::Time::now();
    ekf_path.header.frame_id = "/map";
    ekf_path.poses.push_back(ekf_pose_stamp);
    
    
    ekf_pub.publish(state);            // Publish EKF command
    gps_path_pub.publish(gps_path);    // Publish GPS trajectory
    ekf_path_pub.publish(ekf_path);	// Publish EKF trajectory

    // Quaternion to RPY
    double roll, pitch, yaw;
    // tf::Quaternion q(imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w);
    // tf::Matrix3x3 m(q);
    // m.getRPY(roll, pitch, yaw);


    EKF_log.open("EKF_log.csv", std::ios::out | std::ios::app);
    EKF_log << simtim << " " << ekf_pose_stamp.pose.position.x<< " " << ekf_pose_stamp.pose.position.y << " " 
    << gps_pose_stamp.pose.position.x << " " << gps_pose_stamp.pose.position.y << " " << bicycle_data.angular.y 
    << " " << bicycle_data.angular.z << " " <<(float)gps_data.position_covariance[0]<< " " << (float)gps_data.position_covariance[4]
    << " " << roll << " " << pitch << " " << yaw << std::endl;
    EKF_log.close();
    simtim += dt;
    
    ros::spinOnce();
    loop_rate.sleep();
  }
};

void extended_kalman_filter(sensor_msgs::NavSatFix gps_data, geometry_msgs::Twist gps_pos, sensor_msgs::Imu imu_data, geometry_msgs::Twist bicycle_data, geometry_msgs::Twist* state){

  // Quaternion to RPY
  double roll, pitch, yaw;
  tf::Quaternion q(imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);

  // Setup variables
  float x = state->linear.x;
  float y = state->linear.y;
  float phi = state->angular.z;
  float v0 = state->linear.z;
  Eigen::Vector4f mu(x, y, phi, v0);
  
  float theta = bicycle_data.linear.x;
  float theta_dot = bicycle_data.linear.y;
  float delta = bicycle_data.linear.z;
  float v_encoder = bicycle_data.angular.x;
  // use stm32 imu
  // float a_x = bicycle_data.angular.y;
  // float phi_dot = bicycle_data.angular.z;
  // use PX4 imu
  float phi_dot = imu_data.angular_velocity.z*cosf(roll);
  float a_x = imu_data.linear_acceleration.x + 9.81*sinf(pitch);
  if (phi_dot < 0.00000001) phi_dot = 0;
  if (a_x < 0.00000001) a_x = 0;
  
  float g13 = -v0*dt*sinf(phi);
  float g14 = dt*cosf(phi);
  float g23 = v0*dt*cosf(phi);
  float g24 = dt*sinf(phi);

  float v11 = dt*dt*cosf(phi);
  float v12 = -v0*dt*dt*sinf(phi);
  float v21 = dt*dt*sinf(phi);
  float v22 = v0*dt*dt*cosf(phi);

  // Construct the Jacobian matrices
  // Motion model jacobian matrix

  Eigen::Matrix4f Gt;
  Gt << 1,0,g13,g14,
  	0,1,g23,g24,
   	0,0,1,0,
  	0,0,0,1;

  // Sensor model jacobian matrix
  // Another approach is to replace VtMkVtT with Q (state model noise covariance matrix)
  // and just tweek the Q matrix
  Eigen::MatrixXf Vt(4,2);
  Vt << v11,v12,
   	v21,v22,
  	0,dt,
  	1,0;

  // Covariance matrix of the noise in control space
  Eigen::Matrix2f Mt;
  Mt << 0.0001,0,
        0,0.0001;

  // Prediction (use FCU imu)
  mu(0) = x + v0*dt*cosf(phi);
  mu(1) = y + v0*dt*sinf(phi);
  mu(2) = phi + phi_dot*dt;
  mu(3) = v0 + a_x*dt;
  // Prediiction (use bicycle imu)
//  mu(0) = x + v0*dt*cosf(phi);
//  mu(1) = y + v0*dt*sinf(phi);
//  mu(2) = phi + bicycle_data.angular.y/cosf(roll)*dt;
//  mu(3) = v0 + bicycle_data.angular.z*dt;
  
  //ROS_INFO("predict: %f, %f, v0: %f a: %f, phi_d: %f", mu(0), mu(1), v0, imu_data.linear_acceleration.x,imu_data.angular_velocity.z/cosf(roll));

  static Eigen::Matrix4f covariance_est;
  covariance_est = Gt*covariance_est*Gt.transpose() + Vt*Mt*Vt.transpose();

  // Update
  // Both GPS and IMU data comes in at 50 hz
  // The encoder data is needed from the lower-level controller
  if(PX4_FLAG && BIKE_FLAG){  // @TODO
    // Compute the measurement prediction z_hat = [x, y, v]^T
    Eigen::Vector3f z_hat(mu(0), mu(1), mu(2));
    Eigen::Vector3f z((float)gps_pos.linear.x, (float)gps_pos.linear.y, v_encoder);
    Eigen::MatrixXf H(3,4);
    H << 1,0,0,0,
    	 0,1,0,0,
    	 0,0,0,1;

    //ROS_INFO("z_hat - z: %f %f %f", (z_hat-z)(0), (z_hat-z)(1), (z_hat-z)(2));
    // Compute the sensor measurment covariance matrix
//    float Q[] = {(float)gps_data.position_covariance[0],0,0,
//	         0,(float)gps_data.position_covariance[4],0,
//                 0,0,0.03f}; // @TODO, not sure if it is correct to use the position cov directly
    Eigen::Matrix3f Q;
    Q << 0.001*(float)gps_data.position_covariance[0], 0.0, 0.0,
	 0.0, 0.001*(float)gps_data.position_covariance[4], 0.0,
	 0.0, 0.0, 0.2f;

    // Compute the kalman gain
    Eigen::Matrix3f S;
    Eigen::MatrixXf K(4,3);
    S = H*covariance_est*H.transpose()+Q;
    K = covariance_est*H.transpose()*S.inverse();
    mu = mu + K*(z-z_hat);
    covariance_est = (Eigen::Matrix4f::Identity()-K*H)*covariance_est;
    
    PX4_FLAG = 0;
    BIKE_FLAG = 0;
  }

  // return EKF estimate
  state->linear.x = mu(0);
  state->linear.y = mu(1);
  state->angular.z = mu(2);
  state->linear.z = mu(3);
  
  ROS_INFO("fused: %f, %f, ori: %f, %f", mu(0), mu(1), (float)gps_pos.linear.x, (float)gps_pos.linear.y);
}

//rostopic pub -r 50 /bicycle_data geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
//roslaunch mavros px4.launch fcu_url:=/dev/ttyUSB0:921600
