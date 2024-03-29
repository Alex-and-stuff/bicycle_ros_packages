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
#include <chrono>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#define T_HRZ         3.0f			    // Prediction time horizon
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

// #define K1            30.824f
// #define K2            11.3638f
// #define K3            0.000637f
// #define K1            32.4522f
// #define K2            7.0690f
// #define K3            0.0207f

// #define K1            32.8939f
// #define K2            6.5065f
// #define K3            -0.0010786f
// #define K1            29.6212f
// #define K2            9.1605f
// #define K3            -0.1089f

#define K1            38.9562f
#define K2            11.3425f
#define K3            -0.000976f

// #define K1            42.3960f
// #define K2            9.8507f
// #define K3            -1.5682f

// Self-define structures
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

// Initialize functions
void kernelmain(Control* output_cmd, Control* host_U, State* output_state, int freq, float* cost, Obs* obs_pos, int obs_num);
void trackInit();
//void randomInit(int seed);
void quaternion2RPY(sensor_msgs::Imu input, double *output);
State bicycleModelUpdateOld(State state, Control u);
State bicycleModelUpdate(State state, Control u);

// Global variables
nav_msgs::Odometry pos;
// ros::Publisher markerPub;

// Other params for callback fcn
sensor_msgs::Imu imu_data;
geometry_msgs::Twist command;
sensor_msgs::JointState joint_state;
Obs obs_pos[MAXCLUSTER] = {0};
int obs_num = 0;

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

void trackingObsCallback(const geometry_msgs::PoseArray::ConstPtr& msg){
  obs_num = msg->poses.size();
  for(int i = 0; i < obs_num; i ++){
    Obs obstacle = {float(msg->poses[i].position.x), float(msg->poses[i].position.y), float(msg->poses[i].position.z)};
    obs_pos[i] = obstacle;
  }
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
    // std::cout<<"PUB!!! ("<<obstacle[i].x<<","<<obstacle[i].y<<")"<<std::endl;
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
  geometry_msgs::PoseStamped mpc_prediction_pose_stamp;
  
  double v_no_control = 0;
  double vehicle_output[3] = {0}; // Store in roll / pitch / yaw values
  State start_pos = {0};
  float cost = 1000;
  static float old_cost = 1000;
  int renew_counter = 0;
  
  // Setup publishers
  ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  ros::Publisher path_pub = n.advertise<nav_msgs::Path>("/bike/trajectory",10);
  ros::Publisher mpc_path_pub = n.advertise<nav_msgs::Path>("/bike/mpc_trajectory",10);
  ros::Publisher mpc_prediction_pub = n.advertise<nav_msgs::Path>("/bike/mpc_prediction",10);
  ros::Publisher markerPub = n.advertise<visualization_msgs::MarkerArray>("/detection", 10);

  // Setup subscribers
  ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("/bike/odom", 10, &odomPosCallback);
  ros::Subscriber joint_sub = n.subscribe<sensor_msgs::JointState>("/bike/joint_states", 10, &jointStateCallback);
  ros::Subscriber imu_sub = n.subscribe<sensor_msgs::Imu>("/bike/imu", 10, &imuCallback);
  ros::Subscriber obs_sub = n.subscribe<geometry_msgs::PoseArray>("/obs_pos", 10, &trackingObsCallback);
  
  // Setup MPC variables
  // State initial_state = {150.91,126.71,-1,2};
  State initial_state = {134.273,139.296,-0.65,2};
  State current_state = initial_state;
  Control output_cmd;
  Control host_U[int(N_HRZ)]={0};
  Control former_host_U[int(N_HRZ*0.2)]={0};

  // Other variables
  int counter = 0;
  static double simtim = 0;
  
  // CUDA Initialize track
  trackInit();
  
  // CUDA Initialize the random perturbation
  //randomInit(0);

  // Output .csv files
  std::fstream MPC_log;

  // static float phi_old = 0;

  while (ros::ok())
  {
    // Raise velocity to speed limit before starting MPC control
    if(counter < SPEEDUP_P){
      cmd.linear.x = v_no_control;
      cmd.linear.y = 0;
      cmd_pub.publish(cmd);
      if(v_no_control<CTRL_V){
        v_no_control += double(CTRL_V)/double(SPEEDUP_P);
      }
    }
    else{
      // Record initial state at the start of control
      if(counter == SPEEDUP_P){
        start_pos.x = pos.pose.pose.position.x;
        start_pos.y = pos.pose.pose.position.y;
        
        initial_state.x -= start_pos.x; 
        initial_state.y -= start_pos.y; 
        // phi_old = vehicle_output[2];
        current_state.x =  start_pos.x;
        current_state.y =  start_pos.y;
      }
      // Start MPC
      quaternion2RPY(imu_data, vehicle_output);
      
      // Update current_state with position feedback
      //if(pow(pos.pose.pose.position.x + initial_state.x - current_state.x,2) +
      //pow(pos.pose.pose.position.y + initial_state.y - current_state.y,2) > 0.25){
      //if(pow(current_state.phi - vehicle_output[2],2)>0.2){
        //current_state.x = pos.pose.pose.position.x + initial_state.x; 
        //current_state.y = pos.pose.pose.position.y + initial_state.y; 
        //current_state.phi =  vehicle_output[2];
        //current_state.v0 = (double)joint_state.velocity[2] * WHEEL_R;
      //}

      if(counter%1==0){
        current_state.x = pos.pose.pose.position.x; 
        current_state.y = pos.pose.pose.position.y; 
        current_state.phi =  vehicle_output[2]*cosf(vehicle_output[0]); // yaw
        current_state.v0 = (double)joint_state.velocity[2] * WHEEL_R;
        current_state.theta = vehicle_output[0];  // roll
        current_state.theta_dot = imu_data.angular_velocity.x;
        current_state.delta = (float)joint_state.position[1];
      }

      
      // Obs obs_pos_test[] = {
      // {154 ,126.2 ,1},
      // {153 ,124.5 ,1},
      // // {152 ,120.5 ,1},
      // {128 ,87.5 ,1.0},
      // {152 ,119   ,1},
      // {134, 97, 1}
      // };
      // obs_num = 5;

      // Run 1 MPC time horizon prediction
      auto t_start = std::chrono::steady_clock::now();

      obstacleFrameTrans(obs_pos, current_state);

      // std::cout<<"Phi: "<<current_state.phi<<" Theta: "<< current_state.theta<<std::endl;

      kernelmain(&output_cmd, host_U, &current_state, int(F_STP), &cost, obs_pos, obs_num);
      
      visualization_msgs::MarkerArray clusterMarkers;
      publishMarkers(obs_pos, clusterMarkers);
      markerPub.publish(clusterMarkers);
      // for(int i = 0; i < obs_num; i ++){
      //   std::cout <<"orig Pos: ("<< current_state.x<<" "<< current_state.y<<") "<<obs_pos[i].x<<","<<obs_pos[i].y<<std::endl;
      // }

      auto t_end = std::chrono::steady_clock::now();
      auto t_diff = t_end - t_start;
      // std::cout << std::chrono::duration <double, std::milli> (t_diff).count() << " ms" << std::endl;
      // for(int i = 0; i < obs_num; i ++){
      //   std::cout<<obs_pos[i].x<<" "<<obs_pos[i].y<<" "<<obs_pos[i].r<<std::endl;
      // }

      // MPC error 
      if(output_cmd.vd == 0 && output_cmd.wd){
        ROS_INFO("MPC_ERROR");
        break;
      }
      
      // std::cout<< "[Counter] : "<<counter<<std::endl;
      
      // Publish optimal command calculated by MPC
      cmd.linear.x = output_cmd.vd;
      cmd.linear.y = output_cmd.wd;  //0.37
      // if(cost > old_cost+COST_BUFFER && renew_counter < 0.1*N_HRZ){
      //   cmd.linear.x = former_host_U[renew_counter].vd;
      //   cmd.angular.z = former_host_U[renew_counter].wd;
      //   renew_counter ++;
      //   std::cout<<"BOOM!!!!"<<std::endl;
      // }else{
      //   cmd.linear.x = output_cmd.vd;
      //   cmd.angular.z = output_cmd.wd;    
      //   memcpy(former_host_U, host_U, int(N_HRZ*0.1) * sizeof(Control));
      //   renew_counter = 0;
      //   old_cost = cost;
      // }
      cmd_pub.publish(cmd);
      
      // Store ground truth position to trajectory
      pose_stamp.pose.position.x = pos.pose.pose.position.x;
      pose_stamp.pose.position.y = pos.pose.pose.position.y;
      pose_stamp.pose.position.z = 0;
      pose_stamp.header.stamp=ros::Time::now();
      path.header.frame_id = "/map";
      path.poses.push_back(pose_stamp);
      
      // Store mpc model position to trajectory
      mpc_pose_stamp.pose.position.x = current_state.x;
      mpc_pose_stamp.pose.position.y = current_state.y;
      mpc_pose_stamp.pose.position.z = 0;
      mpc_pose_stamp.header.stamp=ros::Time::now();
      mpc_path.header.frame_id = "/map";
      mpc_path.poses.push_back(mpc_pose_stamp);
      // std::cout<<"theta: "<<current_state.theta<<" theta_dot: "<<current_state.theta_dot<<" delta: "<<current_state.delta<<std::endl;
      // std::cout<<"cost: "<<cost<<std::endl;

      // Store mpc prediction at each iteration
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
        // std::cout<<host_U[i].wd<<std::endl;

        // for(int j=0;j<5;j++){
        //   state_pred = bicycleModelUpdate(state_pred, host_U[i]);
        // }
        // std::cout<<host_U[i].wd<<" ";
      }
      // std::cout<<std::endl;
      mpc_prediction_path.header.frame_id = "/map";

      // Publish topics
      path_pub.publish(path);
      mpc_path_pub.publish(mpc_path);
      mpc_prediction_pub.publish(mpc_prediction_path);
      
      // create a name for the file output [t, x, y, phi, v0, vd, wd, theta, delta]
      MPC_log.open("MPC_log.csv", std::ios::out | std::ios::app);
      MPC_log << simtim << " " << pos.pose.pose.position.x<< " " << pos.pose.pose.position.y << " " 
      << vehicle_output[2] << " " << (double)joint_state.velocity[2] * WHEEL_R << " " <<cmd.linear.x  << " " 
      << cmd.linear.y << " " <<vehicle_output[0]<< " "<<(double)joint_state.position[1]<<" "<<vehicle_output[0]<<" "<<imu_data.angular_velocity.x<< std::endl;
      MPC_log.close();

      // std::cout<<"phi: "<<vehicle_output[2]<<std::endl;
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
  float h       = 0.6;

  // float A[9] = {1.0232, 0.0504, 0, 
  //               0.9326, 1.0232, 0,
  //               0, 0, exp(-10*state.v0/79)};

  // float B[3] = {0.0017, 0.0670, -79*(exp(-10*state.v0/79)-1)/200/state.v0};
  // float B[3] = {0.0017*state.v0, 0.0670*state.v0, -79*(expf(-10*state.v0/79)-1)/200/state.v0};
  float A[9] = {1,        T_STP,    0,
                g/h*T_STP,    1,    0,
                0,            0,  1-state.v0/l_a*T_STP};

  float B[3] = {0, l_a*state.v0*sin(epsilon)*T_STP/l_b/h, T_STP};

  // Full state feedback to compute u_bar
  float delta_d   = 1/std::sin(epsilon)*std::atan(l_b*u.wd/u.vd);
  // float K_gain[3] = {25.1229, 5.2253, 0.016359};
  // float K_gain[3] = {32.4522, 7.0690, 0.0207};
  float K_gain[3] = {K1, K2, K3};
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
  // float K_gain[3] = {-32.4522, -7.0690, -0.00207};
  float K_gain[3] = {K1, K2, K3};
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