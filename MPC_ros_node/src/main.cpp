#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <fstream>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/NavSatFix.h>
#include <chrono>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#define T_HRZ         5.0f			    // Prediction time horizon
#define F_STP         20.0f			    // Frequency of prediction (control freq)
#define T_STP         1/F_STP			  // Period of prediction
#define N_HRZ         T_HRZ*F_STP		// N steps in a time horizon
#define ST_FREQ       100
#define WHEEL_R       0.325f
#define SPEEDUP_P     100
#define CTRL_V        2
#define W_MAX         1.0f
#define W_MIN         -1.0f
#define SCALE_MODEL   1.0f//0.68f //0.72f
#define COST_BUFFER   100.0f
#define DELTA_MAX     0.6f
#define DELTA_MIN     -0.6f
#define MAXCLUSTER    5
#define V_START       1.0f

#define USE_STM_IMU
#define USE_EKF_PHI
// #define USE_POS_BIAS
// #define USE_OBS_DETECT

/* Self-define structures */
struct Control {
	float vd, wd;
};
struct State {
  float x, y, phi, v0, theta, theta_dot, delta;
};
struct Track {
  // Track fcn in the form of by + ax + c =0
  float b, a, c;
};
struct Obs {
  float x, y, r;
};

/* Initialize functions */
void kernelmain(Control* output_cmd, Control* host_U, State* output_state, int freq, float* cost, Obs* obs_pos, int obs_num);
void trackInit();
void WsgToTWD(double raw_lat, double raw_lon, float* output);
void quaternion2RPY(sensor_msgs::Imu input, double *output);
void obstacleFrameTrans(Obs* obs_pos, State &bike_state);
void publishMarkers(Obs *obstacle, visualization_msgs::MarkerArray &clusterMarkers);
State bicycleModelUpdateOld(State state, Control u);
State bicycleModelUpdate(State state, Control u);

/* Global variables */
geometry_msgs::Twist pos;

/* Other params for callback fcn */
sensor_msgs::Imu imu_data;
geometry_msgs::Twist command;
sensor_msgs::JointState joint_state;
bool STATE_FLAG = false;
bool EKF_FLAG = false;
Obs obs_pos[MAXCLUSTER] = {0};
int obs_num = 0;

/* Setup callback function */
void ekfCallback(const geometry_msgs::Twist::ConstPtr& msg){  
  /* msg->linear.x:  pos x
   * msg->linear.y:  pos y
   * msg->angular.z: phi angle
   * msg->linear.z:  velocity */
  pos = *msg;
  EKF_FLAG = true;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
  /* Callback from Pixhwak IMU */
  imu_data = *msg;
}

void bikeStateCallback(const geometry_msgs::Twist::ConstPtr& msg){
  /* Callback from STM32 rosserial */
  joint_state.position.resize(3);
  joint_state.velocity.resize(3);
  joint_state.position[0] = (float)msg->linear.x;   // Stm32 theta angle      (roll angle)
  joint_state.position[1] = (float)msg->linear.y;   // Stm32 theta dot angle  (roll angular velocity)
  joint_state.position[2] = (float)msg->linear.z;   // Stm32 delta angle      (front-fork angle)
  joint_state.velocity[0] = (float)msg->angular.x;  // Stm32 encoder velocity (back-wheel)
  joint_state.velocity[1] = (float)msg->angular.y;  // Stm32 IMU acceleration (acc_x)
  joint_state.velocity[2] = (float)msg->angular.z;  // Stm32 phi angle        (yaw angle)
  STATE_FLAG = true;
}

void trackingObsCallback(const geometry_msgs::PoseArray::ConstPtr& msg){
  /* Callback from Kalman filter tracking */
  obs_num = msg->poses.size();
  for(int i = 0; i < obs_num; i ++){
    Obs obstacle = {float(msg->poses[i].position.x), float(msg->poses[i].position.y), float(msg->poses[i].position.z)};
    obs_pos[i] = obstacle;
  }
}


int main(int argc, char **argv)
{
  /* Initialize ROS */
  ros::init(argc, argv, "mpc_bike_main");
  ros::NodeHandle n;
  ros::NodeHandle private_nh("~");
  ros::Rate loop_rate(int(F_STP));
  
  /* Setup variables */
  geometry_msgs::Twist cmd;
  geometry_msgs::PoseStamped pose_stamp;
  nav_msgs::Path path;
  geometry_msgs::PoseStamped mpc_pose_stamp;
  nav_msgs::Path mpc_path;
  geometry_msgs::PoseStamped mpc_prediction_pose_stamp;
  
  double v_no_control = 0;
  double vehicle_output[3] = {0}; // Store in roll / pitch / yaw values
  State start_pos = {0};
  float cost = 1000;
  static float old_cost = 1000;
  int renew_counter = 0;
  
  /* Setup publishers */
  ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  ros::Publisher path_pub = n.advertise<nav_msgs::Path>("/bike/trajectory",10);
  ros::Publisher mpc_path_pub = n.advertise<nav_msgs::Path>("/bike/mpc_trajectory",10);
  ros::Publisher mpc_prediction_pub = n.advertise<nav_msgs::Path>("/bike/mpc_prediction",10);
  ros::Publisher markerPub = n.advertise<visualization_msgs::MarkerArray>("/detection", 10);
  
  /* Setup subscribers */
  ros::Subscriber ekf_sub = n.subscribe<geometry_msgs::Twist>("bike/ekf_pos", 10, &ekfCallback);
  ros::Subscriber state_sub = n.subscribe<geometry_msgs::Twist>("/output", 10, &bikeStateCallback);
  ros::Subscriber imu_sub = n.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, &imuCallback);
  ros::Subscriber obs_sub = n.subscribe<geometry_msgs::PoseArray>("/obs_pos", 10, &trackingObsCallback);

  /* Setup MPC variables */
  float INIT_X, INIT_Y, INIT_PHI;
  private_nh.getParam("INIT_X",INIT_X);
  private_nh.getParam("INIT_Y",INIT_Y);
  private_nh.getParam("INIT_PHI",INIT_PHI);

  State initial_state = {INIT_X, INIT_Y,INIT_PHI,2};
  State current_state = initial_state;
  Control output_cmd;
  Control host_U[int(N_HRZ)]={0};
  Control former_host_U[int(N_HRZ*0.2)]={0};

  float bias_x = 0, bias_y = 0, bias_counter = 0;

  /* Other variables */
  int counter = 0;
  static double simtim = 0;
  bool MPC_FLAG = false;
  
  /* CUDA Initialize track */
  trackInit();

  /* Output .csv files */
  std::fstream MPC_log;

  /* Start MPC loop */
  while (ros::ok())
  {
    if(!STATE_FLAG&&!EKF_FLAG){
      ROS_INFO("Waiting for topic ...");
    }
    else{
      if(!MPC_FLAG){       
          /* Averages the stationary drift and treats it as position bias */
          #ifdef USE_POS_BIAS   
          bias_counter++;
          bias_x = (bias_x*(bias_counter-1) + initial_state.x - pos.linear.x)/bias_counter;
          bias_y = (bias_y*(bias_counter-1) + initial_state.y - pos.linear.y)/bias_counter;
          // ROS_INFO("Bias: %.3f %.3f", bias_x, bias_y);
          #endif
          /* Start MPC only when vehicle velocity is larger than V_START */
          if(joint_state.velocity[0]>V_START){
            ROS_INFO("======= MPC START! =======");
            MPC_FLAG = true;
          }   
      }
      else{
        quaternion2RPY(imu_data, vehicle_output);
        #ifdef USE_STM_IMU  // Replace theta with stm32 measured theta
        vehicle_output[0] = joint_state.position[0];
        #endif
        #ifdef USE_EKF_PHI  // Replace px4 measured phi with ekf fused phi
        vehicle_output[2] = pos.angular.z;
        #endif

        /* Update every iteration */
        if(counter%1==0){
          current_state.x = pos.linear.x + bias_x;  
          current_state.y = pos.linear.y + bias_y; 
          current_state.v0 = (double)joint_state.velocity[0] * WHEEL_R;
          current_state.delta = (float)joint_state.position[2];
          // current_state.phi = vehicle_output[2];              // yaw *Note that this is the px4's IMU*
          current_state.phi = pos.angular.z;

          #ifdef USE_STM_IMU
          current_state.theta = joint_state.position[0];      
          current_state.theta_dot = joint_state.position[1];
          #endif
          #ifndef USE_STM_IMU
          current_state.theta = vehicle_output[0];           
          current_state.theta_dot = imu_data.angular_velocity.x;
          #endif
        }
        
        /* Run 1 MPC time horizon prediction */
        auto t_start = std::chrono::steady_clock::now();
        obstacleFrameTrans(obs_pos, current_state);

        #ifndef USE_OBS_DETECT
        for(int i =0; i < MAXCLUSTER; i++){
          obs_pos[i].x = 0;
          obs_pos[i].y = 0;
          obs_pos[i].r = 0;
        }
        #endif

        //std::cout<<"Phi: "<<current_state.phi<<" Theta: "<< current_state.theta<<std::endl;

        kernelmain(&output_cmd, host_U, &current_state, int(F_STP), &cost, obs_pos, obs_num);
        
        visualization_msgs::MarkerArray clusterMarkers;
        publishMarkers(obs_pos, clusterMarkers);
        markerPub.publish(clusterMarkers);        
        auto t_end = std::chrono::steady_clock::now();
        auto t_diff = t_end - t_start;
        std::cout << std::chrono::duration <double, std::milli> (t_diff).count() << " ms" << std::endl;

        /* MPC error */
        if(output_cmd.vd == 0 && output_cmd.wd){
          ROS_INFO("MPC_ERROR");
          break;
        }
        
        /* Publish optimal command calculated by MPC */
        cmd.linear.x = output_cmd.vd;
        cmd.linear.y = -output_cmd.wd;  //0.37
        cmd_pub.publish(cmd);
        
        //ROS_INFO("POS: %.3f %.3f CMD: %.3f %.3f Bias: %.3f %.3f", pos.linear.x, pos.linear.y, cmd.linear.x, cmd.angular.z, bias_x, bias_y);

        /* Store EKF position to trajectory */
        pose_stamp.pose.position.x = pos.linear.x + bias_x;
        pose_stamp.pose.position.y = pos.linear.y + bias_y;
        pose_stamp.pose.position.z = 0;
        pose_stamp.header.stamp=ros::Time::now();
        path.header.frame_id = "/map";
        path.poses.push_back(pose_stamp);
        
        /* Store MPC asumed position to trajectory */
        mpc_pose_stamp.pose.position.x = current_state.x;
        mpc_pose_stamp.pose.position.y = current_state.y;
        mpc_pose_stamp.pose.position.z = 0;
        mpc_pose_stamp.header.stamp=ros::Time::now();
        mpc_path.header.frame_id = "/map";
        mpc_path.poses.push_back(mpc_pose_stamp);

        /* Store MPC predictions at each iteration */
        nav_msgs::Path mpc_prediction_path;
        State state_pred = current_state;
        state_pred.x = current_state.x;
        state_pred.y = current_state.y;

        for(int i=0; i<int(N_HRZ); i++){
          mpc_prediction_pose_stamp.pose.position.x = state_pred.x;
          mpc_prediction_pose_stamp.pose.position.y = state_pred.y;
          mpc_prediction_pose_stamp.pose.position.z = 0;
          mpc_prediction_pose_stamp.header.stamp=ros::Time::now();
          mpc_prediction_path.poses.push_back(mpc_prediction_pose_stamp);
          state_pred = bicycleModelUpdate(state_pred, host_U[i]);
        }
        mpc_prediction_path.header.frame_id = "/map";

        /* Publish topics */
        path_pub.publish(path);
        mpc_path_pub.publish(mpc_path);
        mpc_prediction_pub.publish(mpc_prediction_path);
        
        /* create a name for the file output [t, x, y, phi, v0, vd, wd, theta, delta] */
        MPC_log.open("MPC_log.csv", std::ios::out | std::ios::app);
        MPC_log << simtim << " " << pos.linear.x<< " " << pos.linear.y << " " 
        << vehicle_output[2] << " " << (double)joint_state.velocity[0] * WHEEL_R << " " <<cmd.linear.x  << " " 
        << cmd.angular.z << " " <<vehicle_output[0]<< " "<<(double)joint_state.position[2]<< std::endl;
        MPC_log.close();
      }
    }
    
    simtim += T_STP;
    
    ros::spinOnce();
    loop_rate.sleep();
    counter ++;
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

State bicycleModelUpdateOld(State state, Control u) {
/*  Linear bicycle model, which geneates state_dot. Then by integrating the state_dot, 
  we can then get the nest state x_k+1*/
  State state_dot;
  State state_next;
  float Ka = 2.54;
  state_dot.x = state.v0 * cosf(state.phi);
  state_dot.y = state.v0 * sinf(state.phi);
  state_dot.phi = u.wd;
  state_dot.v0 = Ka * state.v0 * (u.vd - state.v0);
  state_next.x = state.x + state_dot.x * T_STP;
  state_next.y = state.y + state_dot.y * T_STP;
  state_next.phi = state.phi + state_dot.phi * T_STP;
  state_next.v0 = state.v0 + state_dot.v0 * T_STP;
  return state_next;
}

State bicycleModelUpdate(State state, Control u) {
/*  Linear bicycle model, which geneates state_dot. Then by integrating the state_dot, 
  we can then get the nest state x_k+1*/
  
  // 1. Compute delta with discrete linear state space model
  //    1.1 Perform Full state feedback to get command u
  //    1.2 Obtain theta[k+1], theta_dot[k+1], delta[k+1]
  //
  // 2. Compute the current phi_dot with delta[k+1] as the actual omega

  // Setup parameters
  float pi      = 3.14159265359;
  float epsilon = 70*pi/180;
  float l_a     = 0.395;
  float l_b     = 1.053;
  float g       = 9.81;
  float Ka      = 2.54;

  float A[9] = {1.0232, 0.0504, 0, 
                0.9326, 1.0232, 0,
                0, 0, exp(-10*state.v0/79)};

  float B[3] = {0.0017, 0.0670, -79*(exp(-10*state.v0/79)-1)/200/state.v0};

  // Full state feedback to compute u_bar
  float delta_d   = 1/std::sin(epsilon)*std::atan(l_b*u.wd/u.vd);
  // float K_gain[3] = {25.1229, 5.2253, 0.016359};
  float K_gain[3] = {32.4522, 7.0690, 0.0207};
  float x[3]      = {state.theta, state.theta_dot, state.delta};
  float x_d[3]    = {(-u.vd*u.vd*std::sin(epsilon))/(g*l_b)*delta_d, 0, delta_d};
  float u_d       = (u.vd/l_a)*delta_d*SCALE_MODEL;
  float u_bar     = 0;
  float u_fb      = 0;

  for(int i=0; i<3; i++){
    x[i] = x[i] - x_d[i];  
  }
  for(int i=0; i<3; i++){
    u_fb -= K_gain[i]*x[i]; 
  }
  u_bar = (u_fb + u_d);

  // Obtain state[k+1]
  State state_next      = {0};
  State state_dot       = {0};
  state_dot.x           = state.v0 * cosf(state.phi);
  state_dot.y           = state.v0 * sinf(state.phi);
  state_dot.v0          = Ka * state.v0 * (u.vd - state.v0);
  state_next.theta      = A[0]*state.theta + A[1]*state.theta_dot + A[2]*state.delta + B[0]*u_bar;
  state_next.theta_dot  = A[3]*state.theta + A[4]*state.theta_dot + A[5]*state.delta + B[1]*u_bar;
  state_next.delta      = A[6]*state.theta + A[7]*state.theta_dot + A[8]*state.delta + B[2]*u_bar;
  if(state_next.delta > DELTA_MAX){state_next.delta = DELTA_MAX;}
  if(state_next.delta < DELTA_MIN){state_next.delta = DELTA_MIN;}
  state_next.x          = state.x + state_dot.x * T_STP;
  state_next.y          = state.y + state_dot.y * T_STP;
  state_next.v0         = state.v0 + state_dot.v0 * T_STP;
  state_dot.phi         = std::tan(state_next.delta*std::sin(epsilon))/l_b*state_next.v0;
  // if(state_dot.phi > W_MAX){state_dot.phi = W_MAX;}
  // if(state_dot.phi < W_MIN){state_dot.phi = W_MIN;}
  state_next.phi        = state.phi + state_dot.phi * T_STP;
  
  return state_next;
}

State bicycleModelUpdateConti(State state, Control u) {
/*  Linear bicycle model, which geneates state_dot. Then by integrating the state_dot, 
  we can then get the nest state x_k+1*/
  
  // 1. Compute delta with discrete linear state space model
  //    1.1 Perform Full state feedback to get command u
  //    1.2 Obtain theta[k+1], theta_dot[k+1], delta[k+1]
  //
  // 2. Compute the current phi_dot with delta[k+1] as the actual omega

  // Setup parameters
  float pi      = 3.14159265359;
  float epsilon = 70*pi/180;
  float l_a     = 0.395;
  float l_b     = 1.053;
  float g       = 9.81;
  float Ka      = 2.54;
  float h       = 0.45;

  float A[9] = {0,   1,  0,
                g/h, 0,  0,
                0,   0, -state.v0/l_a};

  float B[3] = {0, l_a*state.v0*std::sin(epsilon)/l_b/h, 1};

  // Full state feedback to compute u_bar
  float delta_d   = 1/std::sin(epsilon)*std::atan(l_b*u.wd/u.vd);
  float K_gain[3] = {-32.4522, -7.0690, -0.00207};
  float x[3]      = {state.theta, state.theta_dot, state.delta};
  float x_d[3]    = {(-u.vd*u.vd*std::sin(epsilon))/(g*l_b)*delta_d, 0, delta_d};
  float u_d       = (u.vd/l_a)*delta_d;
  float u_bar     = 0;
  float u_fb      = 0;

  for(int i=0; i<3; i++){
    x[i] = x[i] - x_d[i];  
  }
  for(int i=0; i<3; i++){
    u_fb -= K_gain[i]*x[i]; 
  }
  u_bar = u_fb + u_d;

  // Obtain state[k+1]
  State state_next      = {0};
  State state_dot       = {0};
  state_dot.x           = state.v0 * cosf(state.phi);
  state_dot.y           = state.v0 * sinf(state.phi);
  state_dot.v0          = Ka * state.v0 * (u.vd - state.v0);
  state_dot.theta       = A[0]*state.theta + A[1]*state.theta_dot + A[2]*state.delta + B[0]*u_bar;
  state_dot.theta_dot   = A[3]*state.theta + A[4]*state.theta_dot + A[5]*state.delta + B[1]*u_bar;
  state_dot.delta       = A[6]*state.theta + A[7]*state.theta_dot + A[8]*state.delta + B[2]*u_bar;
  state_next.x          = state.x + state_dot.x * T_STP;
  state_next.y          = state.y + state_dot.y * T_STP;
  state_next.v0         = state.v0 + state_dot.v0 * T_STP;
  state_next.theta      = state.theta + state_dot.theta * T_STP;
  state_next.theta_dot  = state.theta_dot + state_dot.theta_dot * T_STP;
  state_next.delta      = state.delta + state_dot.delta * T_STP;
  state_dot.phi         = std::tan(state_next.delta*std::sin(epsilon))/l_b*state_next.v0;
  // if(state_dot.phi > W_MAX){state_dot.phi = W_MAX;}
  // if(state_dot.phi < W_MIN){state_dot.phi = W_MIN;}
  state_next.phi        = state.phi + state_dot.phi * T_STP;
  
  return state_next;
}

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

void obstacleFrameTrans(Obs* obs_pos, State &bike_state){
  /* Transform the detected object coordinates from the "camera" frame to the "global frame"
   * Might need to compensate tilt(theta) of phi*/
  float x = 0;
  float y = 0;
  float x_i;
  float y_i;
  float z_i = 0;
  float cam_base_dist = 0.7481;
  float phi = bike_state.phi - 3.1415926/2;
  float theta = bike_state.theta;
  float x_b = bike_state.x;
  float y_b = bike_state.y;
  for(int i = 0; i < obs_num; i++){
    x_i = obs_pos[i].x;
    y_i = obs_pos[i].y;
    /* Obstacle (from sensor) to global coordinates derived through HTM matrices 
     * 1. Translation from origin to global coordinates (x_b, y_b)
     * 2. Rotation along the z-axis to the bike frame by phi
     * 3. Rotation along the z-axis to the sensor frame by -pi/2 
     * 4. Translation along the y-axis to the sensor frame
     * 5. Rotation along the y-axis by theta (roll angle of bicycle) */
    x = cosf(phi)*cosf(theta)*x_i - sinf(phi)*y_i - cosf(phi)*sinf(theta)*z_i - sinf(phi)*cam_base_dist + x_b;
    y = sinf(phi)*cosf(theta)*x_i + cosf(phi)*y_i - sinf(phi)*sinf(theta)*z_i + cosf(phi)*cam_base_dist + y_b;
    obs_pos[i].x = x;
    obs_pos[i].y = y;
  }
}

void publishMarkers(Obs *obstacle, visualization_msgs::MarkerArray &clusterMarkers){
  

  for (int i = 0; i < obs_num; i++) {
    std::cout<<"PUB!!! ("<<obstacle[i].x<<","<<obstacle[i].y<<")"<<std::endl;
    visualization_msgs::Marker m;
    m.id = i;
    m.type = visualization_msgs::Marker::CYLINDER;
    m.header.frame_id = "/map";

    /* Resize the marker according to its decay rate */
    m.scale.x = obstacle[i].r;
    m.scale.y = obstacle[i].r;
    // m.scale.x = kfPredictions[i].z;
    // m.scale.y = kfPredictions[i].z;
    m.scale.z = 0.3;
    m.action = visualization_msgs::Marker::ADD;
    m.color.a = 0.5;
    m.color.r = 240;
    m.color.g = 10;
    m.color.b = 0;
    m.pose.orientation.x = 0.0;
    m.pose.orientation.y = 0.0;
    m.pose.orientation.z = 0.0;
    m.pose.orientation.w = 1.0;

    m.pose.position.x = obstacle[i].x;
    m.pose.position.y = obstacle[i].y;
    m.pose.position.z = 0;

    clusterMarkers.markers.push_back(m);
  }

  for(int i = obs_num; i < 5; i++){
    visualization_msgs::Marker m;
    m.id = i;
    m.action = visualization_msgs::Marker::DELETE;
    clusterMarkers.markers.push_back(m);
  }
  // std::cout<<"size:"<<clusterMarkers.markers.size()<<std::endl;
  // markerPub.publish(clusterMarkers);
}