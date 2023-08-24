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
#include <cmath>


using namespace Eigen;

//#define ORIG_LON 1.0f      // define longitude origin 
//#define ORIG_LAT 1.0f      // define latitude origin
#define FREQ     30.0f    // loop_rate
#define dt       1.0f/FREQ // time of 1 loop
// #define USE_STM_DATA

void extended_kalman_filter(sensor_msgs::NavSatFix gps_data, geometry_msgs::Twist gps_pos, geometry_msgs::Twist gps_stm_pos, 
  sensor_msgs::Imu imu_data, geometry_msgs::Twist bicycle_data, geometry_msgs::Twist* state);
void WsgToTWD(double raw_lat, double raw_lon, float* output);

/* Global variables */
sensor_msgs::NavSatFix gps_data; 
geometry_msgs::Twist gps_pos;
geometry_msgs::Twist gps_stm_pos;
sensor_msgs::Imu imu_data;
geometry_msgs::Twist ekf_pos;
geometry_msgs::Twist bicycle_data;

/* Global flags for separate subscribers */
volatile bool PX4_FLAG = 0;
volatile bool STM_FLAG = 0;
volatile bool BIKE_FLAG = 0;
bool START_EKF = 0;

/* Setup callback functions */
void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
  static float x_old = 0;
  static float y_old = 0;
  gps_data = *msg;
  float pos[2] = {0};
  double lat = msg->latitude;
  double lon = msg->longitude;
  WsgToTWD(lat, lon, pos);
  gps_pos.linear.x = pos[0];
  gps_pos.linear.y = pos[1];

  if(std::isfinite(gps_pos.linear.x)&&gps_pos.linear.x<1000&&gps_pos.linear.x>-1000){x_old = gps_pos.linear.x;}
  else{gps_pos.linear.x= x_old;}
  if(std::isfinite(gps_pos.linear.y)&&gps_pos.linear.y<1000&&gps_pos.linear.y>-1000){y_old = gps_pos.linear.y;}
  else{gps_pos.linear.y= y_old;}
  
  PX4_FLAG = 1;
}

void stmGPSCallback(const geometry_msgs::Twist::ConstPtr& msg){
  /* ========================
   * linear.x:  gps latitude
   * linear.y:  gps longitude
   * linear.z:  gps horizontal accuracy
   * angular.x: pos x (converted)
   * angular.y: pos y (converted)
   * angular.z: gps vertical accuracy | @TODO change to "new-gps-data flag"
   * ======================== */
  gps_stm_pos = *msg;

  static float x_old = 0;
  static float y_old = 0;
  float pos[2] = {0};
  double lat = msg->linear.x;
  double lon = msg->linear.y;
  WsgToTWD(lat, lon, pos);
  gps_stm_pos.linear.x = pos[0];
  gps_stm_pos.linear.y = pos[1];

  if(std::isfinite(gps_stm_pos.linear.x)&&gps_stm_pos.linear.x<1000&&gps_stm_pos.linear.x>-1000){x_old = gps_stm_pos.linear.x;}
  else{gps_stm_pos.linear.x= x_old;}
  if(std::isfinite(gps_stm_pos.linear.y)&&gps_stm_pos.linear.y<1000&&gps_stm_pos.linear.y>-1000){y_old = gps_stm_pos.linear.y;}
  else{gps_stm_pos.linear.y= y_old;}
  if(gps_stm_pos.angular.z = 1){
    STM_FLAG = 1;
  }
}


/* Get IMU data from the PX4 FCU */
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
  imu_data = *msg;
}

/* Get bicycle data from stm32 through rosserial */
void bicycleCallback(const geometry_msgs::Twist::ConstPtr& msg){
  /* ========================
   * linear.x:  theta
   * linear.y:  theta_dot
   * linear.z:  delta
   * angular.x: v_encoder
   * angular.y: acceleration_x
   * angular.z: omega 
   * ======================== */
  bicycle_data = *msg;
  static float theta_old = 0;
  static float theta_dot_old = 0;
  static float delta_old = 0;
  static float v_encoder_old = 0;
  static float a_old = 0;
  BIKE_FLAG = 1;
  if(std::isfinite(bicycle_data.linear.x)&&bicycle_data.linear.x<10&&bicycle_data.linear.x>-10){theta_old = bicycle_data.linear.x;}
  else{bicycle_data.linear.x = theta_old;}
  if(std::isfinite(bicycle_data.linear.y)&&bicycle_data.linear.y<10&&bicycle_data.linear.y>-10){theta_dot_old = bicycle_data.linear.y;}
  else{bicycle_data.linear.y = theta_dot_old;}
  if(std::isfinite(bicycle_data.linear.z)&&bicycle_data.linear.z<10&&bicycle_data.linear.z>-10){delta_old = bicycle_data.linear.z;}
  else{bicycle_data.linear.z = delta_old;}
  if(std::isfinite(bicycle_data.angular.x)&&bicycle_data.angular.x<10&&bicycle_data.angular.x>-10){v_encoder_old = bicycle_data.angular.x;}
  else{bicycle_data.angular.x = v_encoder_old;}
  if(std::isfinite(bicycle_data.angular.y)&&bicycle_data.angular.y<10&&bicycle_data.angular.y>-10){a_old = bicycle_data.angular.y;}
  else{bicycle_data.angular.y = a_old;}
}
  
int main(int argc, char** argv){
  /* Setup ros node */
  ros::init(argc, argv, "px4_ekf_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  ros::Rate loop_rate(FREQ); 
  
  geometry_msgs::PoseStamped gps_pose_stamp;
  geometry_msgs::PoseStamped gps_stm_pose_stamp;
  geometry_msgs::PoseStamped ekf_pose_stamp;
  nav_msgs::Path gps_path;
  nav_msgs::Path gps_stm_path;
  nav_msgs::Path ekf_path;
 
  /* Define the subscriber to odometry */
  ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, &imuCallback);
  ros::Subscriber gps_sub = nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 10, &gpsCallback);
  ros::Subscriber bicycle_sub = nh.subscribe<geometry_msgs::Twist>("/output", 10, &bicycleCallback);
  ros::Subscriber stm_gps_sub = nh.subscribe<geometry_msgs::Twist>("/stm_gps", 10, &stmGPSCallback);
  
  /* Define the publishers for bicycle conroller */
  ros::Publisher ekf_pub = nh.advertise<geometry_msgs::Twist>("bike/ekf_pos", 10);
  ros::Publisher gps_path_pub = nh.advertise<nav_msgs::Path>("/bike/gps_trajectory",10);
  ros::Publisher gps_stm_path_pub = nh.advertise<nav_msgs::Path>("/bike/gps_stm_trajectory",10);
  ros::Publisher ekf_path_pub = nh.advertise<nav_msgs::Path>("/bike/ekf_trajectory",10);
  
  /* Setup variables */
  float INIT_X, INIT_Y,INIT_PHI;
  private_nh.getParam("INIT_X",INIT_X);
  private_nh.getParam("INIT_Y",INIT_Y);
  private_nh.getParam("INIT_PHI",INIT_PHI);
  geometry_msgs::Twist state;
  float init_state[] = {INIT_X,INIT_Y,INIT_PHI,0};
  state.linear.x = init_state[0];
  state.linear.y = init_state[1];
  state.angular.z = init_state[2];
  state.linear.z = init_state[3];
  //123.795  55.916
//  state.linear.x = 130.0;
//  state.linear.y = 57.0;
//  state.angular.z = 2.7;
//  state.linear.z = 0.0;
  int counter = 0;
  std::fstream EKF_log;
  static double simtim = 0;
  
  /* Wait until essential topics are recieved */
  while(ros::ok() && !START_EKF){
    if(BIKE_FLAG && PX4_FLAG){
    START_EKF = 1;
    ROS_INFO("== STM32 rosserial data recieved! ==");
    ROS_INFO("==      PX4 FCU data recieved!    ==");
    }
    counter ++; // limits the rate of the ROS_INFO print
    if(counter > 100){
      if(!PX4_FLAG){ROS_INFO("[ERROR]: No PX4 FCU data");}
      if(!BIKE_FLAG){ROS_INFO("[ERROR]: No STM32 rosserial data");}
      counter = 0;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  if(START_EKF){ROS_INFO("==            EKF START!!         ==");}
  
  /* Execute ekf filter and publish the fused state */
  while (ros::ok())
  {
    /* Assure Phi estimate is the initial value when the vehicle starts and is not altered by gps drift */
    if(bicycle_data.angular.x < 1){
      state.angular.z = init_state[2];
    }

    if(START_EKF){
    	extended_kalman_filter(gps_data, gps_pos, gps_stm_pos, imu_data, bicycle_data, &state);
      
    }

    /* Publish trajectories */
    gps_pose_stamp.pose.position.x = gps_pos.linear.x;
    gps_pose_stamp.pose.position.y = gps_pos.linear.y;
    gps_pose_stamp.pose.position.z = 0;
    gps_pose_stamp.header.stamp=ros::Time::now();
    gps_path.header.frame_id = "/map";
    gps_path.poses.push_back(gps_pose_stamp);

    gps_stm_pose_stamp.pose.position.x = gps_stm_pos.linear.x;
    gps_stm_pose_stamp.pose.position.y = gps_stm_pos.linear.y;
    gps_stm_pose_stamp.pose.position.z = 0;
    gps_stm_pose_stamp.header.stamp=ros::Time::now();
    gps_stm_path.header.frame_id = "/map";
    gps_stm_path.poses.push_back(gps_stm_pose_stamp);
    
    if(START_EKF){
      tf2::Quaternion yawQuaternion;
      yawQuaternion.setRPY( 0, 0, state.angular.z );

      ekf_pose_stamp.pose.position.x = state.linear.x;
      ekf_pose_stamp.pose.position.y = state.linear.y;
      ekf_pose_stamp.pose.position.z = 0;
      ekf_pose_stamp.pose.orientation.x = yawQuaternion.getX();
      ekf_pose_stamp.pose.orientation.y = yawQuaternion.getY();
      ekf_pose_stamp.pose.orientation.z = yawQuaternion.getZ();
      ekf_pose_stamp.pose.orientation.w = yawQuaternion.getW();

      ekf_pose_stamp.header.stamp=ros::Time::now();
      ekf_path.header.frame_id = "/map";
      ekf_path.poses.push_back(ekf_pose_stamp);
      
      ekf_pub.publish(state);                     // Publish EKF command
      ekf_path_pub.publish(ekf_path);	            // Publish EKF trajectory
    }
    
    gps_path_pub.publish(gps_path);             // Publish GPS trajectory
    gps_stm_path_pub.publish(gps_stm_path);     // Publish GPS trajectory

    /* Quaternion to RPY */
    double roll, pitch, yaw;
    tf::Quaternion q(imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);

    /* Save results to a .csv file */
    EKF_log.open("EKF_log4.csv", std::ios::out | std::ios::app);
    EKF_log << simtim << " " << ekf_pose_stamp.pose.position.x<< " " << ekf_pose_stamp.pose.position.y << " " 
    << gps_pose_stamp.pose.position.x << " " << gps_pose_stamp.pose.position.y<< gps_stm_pose_stamp.pose.position.x << " " << gps_stm_pose_stamp.pose.position.y << " " << bicycle_data.angular.y 
    << " " << bicycle_data.linear.x<<" "<<bicycle_data.linear.y<<" "<<bicycle_data.linear.z<<" "<<bicycle_data.angular.x<<" "<<bicycle_data.angular.y << " " <<(float)gps_data.position_covariance[0]<< " " << (float)gps_data.position_covariance[4]
    << " " << roll << " " << pitch << " " << yaw << std::endl;
    EKF_log.close();
    simtim += dt;
    
    /* Callback and wait for loop rate if finished early */
    ros::spinOnce();
    loop_rate.sleep();
  }
};

void extended_kalman_filter(sensor_msgs::NavSatFix gps_data, geometry_msgs::Twist gps_pos, geometry_msgs::Twist gps_stm_pos, 
  sensor_msgs::Imu imu_data, geometry_msgs::Twist bicycle_data, geometry_msgs::Twist* state){

  /* Quaternion to RPY (get theta(roll) and phi(yaw) of the bicycle) */
  double roll, pitch, yaw;
  tf::Quaternion q(imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);

  /* Setup variables */
  float x = state->linear.x;
  float y = state->linear.y;
  float phi = state->angular.z;
  float v0 = state->linear.z;
  Eigen::Vector4f mu(x, y, phi, v0);
  
  float theta = bicycle_data.linear.x;
  float theta_dot = bicycle_data.linear.y;
  float delta = bicycle_data.linear.z;
  float v_encoder = bicycle_data.angular.x;

  /* use stm32 imu */
  #ifdef USE_STM_DATA
  float a_x = bicycle_data.angular.y;
  float phi_dot = bicycle_data.angular.z;
  #endif

  /* use PX4 imu */
  #ifndef USE_STM_DATA
  float phi_dot = imu_data.angular_velocity.z*cosf(roll);
  float a_x = imu_data.linear_acceleration.x + 9.81*sinf(pitch);
  #endif
  
  float g13 = -v0*dt*sinf(phi);
  float g14 = dt*cosf(phi);
  float g23 = v0*dt*cosf(phi);
  float g24 = dt*sinf(phi);

  float v11 = dt*dt*cosf(phi);
  float v12 = -v0*dt*dt*sinf(phi);
  float v21 = dt*dt*sinf(phi);
  float v22 = v0*dt*dt*cosf(phi);

  /* Construct the motion model jacobian matrix */
  Eigen::Matrix4f Gt;
  Gt << 1,0,g13,g14,
  	    0,1,g23,g24,
   	    0,0,1,0,
  	    0,0,0,1;

  /* Construct the sensor model jacobian matrix
   * Here we replace Q(state model noise covariance matrix) with VtMkVtT 
   * to account for the bicycle unlinearity caused by acceleration and omega */
  Eigen::MatrixXf Vt(4,2);
  Vt << v11,v12,
   	    v21,v22,
  	    0,dt,
  	    1,0;

  /* Covariance matrix of the noise in control space (non-constant) */
  Eigen::Matrix2f Mt;
  // Mt << 0.01,0,
  //       0,0.01;
  float alpha1 = 0.001; //0.03
  float alpha2 = 0.0001;
  Mt << alpha1+alpha2*phi_dot*phi_dot,0,
        0,alpha1+alpha2*phi_dot*phi_dot;

  /* Prediiction (use bicycle imu) */
  #ifdef USE_STM_DATA
  mu(0) = x + v0*dt*cosf(phi);
  mu(1) = y + v0*dt*sinf(phi);
  mu(2) = phi + bicycle_data.angular.y/cosf(roll)*dt; 
  mu(3) = v0 + bicycle_data.angular.z*dt;
  #endif
  /* Prediction (use FCU imu) */
  #ifndef USE_STM_DATA
  mu(0) = x + v0*dt*cosf(phi);
  mu(1) = y + v0*dt*sinf(phi);
  mu(2) = phi + phi_dot*dt;
  mu(3) = v0 + a_x*dt;
  #endif
  
  /* Compute the covariance estimate from the prediction model */
  static Eigen::Matrix4f covariance_est = (Eigen::Matrix4f() << 0.5, 0, 0, 0, 
                                                                0, 0.5, 0, 0, 
                                                                0, 0, 0.3, 0,
                                                                0, 0, 0, 0.1).finished();
  covariance_est = Gt*covariance_est*Gt.transpose() + Vt*Mt*Vt.transpose();


  /* <======================= Update =======================> */
  static int the_counter = 0;

  /* Update with the px4 GPS data */
  if(PX4_FLAG && BIKE_FLAG && the_counter < 5){  // @TODO, not the best way to implement the update process
    /* Compute the measurement prediction z_hat = [x, y, v]^T */
    Eigen::Vector3f z_hat(mu(0), mu(1), mu(3));
    Eigen::Vector3f z((float)gps_pos.linear.x, (float)gps_pos.linear.y, v_encoder);
    Eigen::MatrixXf H(3,4);
    H << 1,0,0,0,
    	   0,1,0,0,
    	   0,0,0,1;

    /* Compute the sensor measurment covariance matrix */
    Eigen::Matrix3f Q;
    Q << (float)gps_data.position_covariance[0], 0.0, 0.0,
	        0.0, (float)gps_data.position_covariance[4], 0.0,
	        0.0, 0.0, 0.2f;


    /* Compute the kalman gain K */
    Eigen::Matrix3f S;
    Eigen::MatrixXf K(4,3);
    S = H*covariance_est*H.transpose()+Q;
    K = covariance_est*H.transpose()*S.inverse();
    mu = mu + K*(z-z_hat);
    covariance_est = (Eigen::Matrix4f::Identity()-K*H)*covariance_est;
    //std::cout << "z-z_hat: "<<(z-z_hat)(0)<<" "<<(z-z_hat)(1)<<" "<<(z-z_hat)(2)<<std::endl;
    
    PX4_FLAG = 0;
    BIKE_FLAG = 0;
    the_counter++;
  }
  /* Update with the STM32 GPS data */
  if(the_counter >= 5 && STM_FLAG){  // @TODO, not the best way to implement the update process
  // if(STM_FLAG){
    the_counter = 0;
    //ROS_INFO("STM UPDATE!!!");

    /* Compute the measurement prediction z_hat = [x, y, v]^T */
    Eigen::Vector3f z_hat(mu(0), mu(1), mu(3));
    Eigen::Vector3f z((float)gps_stm_pos.linear.x, (float)gps_stm_pos.linear.y, v_encoder);
    Eigen::MatrixXf H(3,4);
    H << 1,0,0,0,
    	   0,1,0,0,
    	   0,0,0,1;

    Eigen::Matrix3f Q;
    float CEP_SCALAR = 2.0;
    float gps_variance = (float)gps_stm_pos.linear.z/1.1774*(float)gps_stm_pos.linear.z/1.1774*CEP_SCALAR;
    Q <<  gps_variance, 0.0, 0.0,  // @TODO, Change to actual gps covariance using COP
	        0.0, gps_variance, 0.0,
	        0.0, 0.0, 0.2f;

    // float gps_variance = gps_stm_pos.linear.z*gps_stm_pos.linear.z*2.198*10/0.2+0.8f;
    // gps_variance = gps_stm_pos.linear.z/1.1774*gps_stm_pos.linear.z/1.1774;
    // Q(0,0) = gps_variance;
    // Q(1,1) = gps_variance;
    // std::cout << "Var: " << (float)gps_stm_pos.linear.z <<std::endl;


    /* Compute the kalman gain */
    Eigen::Matrix3f S;
    Eigen::MatrixXf K(4,3);
    S = H*covariance_est*H.transpose()+Q;
    K = covariance_est*H.transpose()*S.inverse();
    mu = mu + K*(z-z_hat);
    covariance_est = (Eigen::Matrix4f::Identity()-K*H)*covariance_est;

    STM_FLAG = 0;
  }
  if(the_counter >= 5 && !STM_FLAG){
    the_counter = 0;
  }

  /* return EKF estimate */
  state->linear.x = mu(0);
  state->linear.y = mu(1);
  state->angular.z = mu(2);
  state->linear.z = mu(3);
  //ROS_INFO("theta: %f, theta_dot: %f, delta: %f, v_encoder: %f, x: %f, y: %f", theta, theta_dot, delta, v_encoder, x, y);
  //ROS_INFO("fused: %f, %f, ori: %f, %f, cov: %f, %f, rpy: %f, %f, %f, z3: %f", mu(0), mu(1), 
  //(float)gps_pos.linear.x, (float)gps_pos.linear.y, (float)gps_data.position_covariance[0], (float)gps_data.position_covariance[4],
  //roll, pitch, yaw, v_encoder);
}

/* Transform coordinates from WGS84 (lon/lat) to TWD97 representation */
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