#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <math.h>

#define WHEEL_R 0.325f
#define GRAVITY 9.81f
#define EPSILON 70
#define L_B 1.023273f
#define L_A 0.418087f


// Global variables - Ros Param
std::string command_topic_name;
std::string imu_topic_name;
std::string joint_topic_name;
double Kp, Ki, Kd, Ka, V_DIFF, W_DIFF, THETA_COMP_DIFF;

// Other params for callback fcn
sensor_msgs::Imu imu_data;
geometry_msgs::Twist command;
sensor_msgs::JointState joint_state;

// Predefine functions
void PIDControl(sensor_msgs::Imu imu_data, geometry_msgs::Twist command, sensor_msgs::JointState joint_state, double* orientation, double *output);
void PIDControlold(sensor_msgs::Imu imu_data, geometry_msgs::Twist command, sensor_msgs::JointState joint_state, double* orientation, double *output);
void quaternion2RPY(sensor_msgs::Imu input, double *output);

// Setup callback functions
void commandCallback(const geometry_msgs::Twist::ConstPtr& msg){
  command = *msg;    
  // Convert speed from m/s to rad/s
  command.linear.x = command.linear.x / WHEEL_R;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
  imu_data = *msg;
}

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg){
  // msg->position[1] = delta angle (steering angle response)
  joint_state = *msg;
  //std::cout<<joint_state.position[0]<<std::endl;
}
  
int main(int argc, char** argv){
  // Setup ros node
  ros::init(argc, argv, "bicycle_pid_controller");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  ros::Rate loop_rate(100);
  
  // Get ros params
  private_nh.getParam("command_topic_name", command_topic_name);
  private_nh.getParam("imu_topic_name", imu_topic_name);
  private_nh.getParam("joint_topic_name", joint_topic_name);
  private_nh.getParam("Kp",Kp);
  private_nh.getParam("Ki",Ki);
  private_nh.getParam("Kd",Kd);
  private_nh.getParam("Ka",Ka);
  private_nh.getParam("V_DIFF",V_DIFF);
  private_nh.getParam("W_DIFF",W_DIFF);
  private_nh.getParam("THETA_COMP_DIFF",THETA_COMP_DIFF);
  
  // Define the subscriber to odometry
  ros::Subscriber cmd_sub = nh.subscribe<geometry_msgs::Twist>(command_topic_name, 10, &commandCallback);
  ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>(imu_topic_name, 10, &imuCallback);
  ros::Subscriber joint_sub = nh.subscribe<sensor_msgs::JointState>(joint_topic_name, 10, &jointStateCallback);
  
  // Define the publishers for bicycle conroller
  ros::Publisher vel_pub = nh.advertise<std_msgs::Float64>("bike/back_wheel_controller/command", 10);
  ros::Publisher delta_pub = nh.advertise<std_msgs::Float64>("bike/front_fork_controller/command", 10);
  
  // Variables
  double vehicle_orientation[3]= {0};
  double vehicle_output[2] = {0};


  while (ros::ok())
  {
    /*if(command.linear.x > 1.5 / WHEEL_R){
      PIDControl(imu_data, command, joint_state, vehicle_orientation, vehicle_output);
    }  
    std_msgs::Float64 vd;
    std_msgs::Float64 wd;
    vd.data = vehicle_output[0];
    wd.data = vehicle_output[1];*/
    
    std_msgs::Float64 vd;
    std_msgs::Float64 wd;
    vd.data = command.linear.x;
    wd.data = command.linear.y;
    
    if(command.linear.x * WHEEL_R > 1.5){
      PIDControlold(imu_data, command, joint_state, vehicle_orientation, vehicle_output);
      // PIDControl(imu_data, command, joint_state, vehicle_orientation, vehicle_output);
   
      vd.data = vehicle_output[0];
      wd.data = vehicle_output[1];
    }
    //std::cout<<"v: "<<vd.data<<" wd: "<<wd.data<<" theta: "<<vehicle_orientation[0]<<std::endl;
    vel_pub.publish(vd);
    delta_pub.publish(wd);    
    
    ros::spinOnce();
    loop_rate.sleep();
  }
};


void PIDControl(sensor_msgs::Imu imu_data, geometry_msgs::Twist command, sensor_msgs::JointState joint_state, double* orientation, double *output){
  quaternion2RPY(imu_data, orientation);
  double theta_dot = imu_data.angular_velocity.x;
  double vd = command.linear.x * WHEEL_R;
  double wd = command.linear.y;
  double theta = orientation[0];
  double a = imu_data.linear_acceleration.x;
  
  double delta = (double)joint_state.position[1];
  double v0 = (double)joint_state.velocity[2] * WHEEL_R;
  double delta_d = 1/std::sin(EPSILON)*std::atan(L_B*wd/vd);
  
  // Compute compensation tilt angle theta_comp (consider accelleration)
  // double theta_comp = -(L_A*std::sin(EPSILON)*vd*WHEEL_R)/(L_B*GRAVITY)*(vd*WHEEL_R/L_A+(Ka*(vd-v0)*WHEEL_R))*wd;
  double theta_comp = -(L_A*std::sin(EPSILON)*vd)/(L_B*GRAVITY)*(vd/L_A+(Ka*(vd-v0)))*wd;
  
  std::cout<<"PID: vd: "<<vd<<" wd: "<<wd<<" delta_d: "<<delta_d<< " comp: "<<theta_comp<<" theta: "<<theta<<" delta: "<<delta<<" est a: "<<Ka*(vd-v0)<<std::endl;
  
  // Perform PD control (feedback control)
  // double delta_cmd = Kp * (theta - Ki * theta_comp) + Kd * theta_dot + delta_d;
  // double delta_cmd = Kp * (theta-theta_comp) + Kd * theta_dot;
  // double delta_cmd = Kp * (theta) + Kd * theta_dot;
  double delta_cmd = Kp * (theta) + Kd * theta_dot + delta_d;
  if(abs(theta)>1.0){
    delta_cmd = 0;
  }
  double vel_cmd = vd / WHEEL_R;
  
  output[0] = vel_cmd;
  output[1] = delta_cmd;
}

void PIDControlold(sensor_msgs::Imu imu_data, geometry_msgs::Twist command, sensor_msgs::JointState joint_state, double* orientation, double *output){
  quaternion2RPY(imu_data, orientation);
  double theta_dot = imu_data.angular_velocity.x;
  double vd = command.linear.x* WHEEL_R;
  double wd = command.linear.y;
  double theta = orientation[0];
  double a = imu_data.linear_acceleration.x;
  
  double delta = (double)joint_state.position[1];
  double v0 = (double)joint_state.velocity[2]* WHEEL_R;
  
  
  // Confine the steering angle difference to some designated value
  /*static double wd_old = 0;
  if(wd > (wd_old + W_DIFF)){wd = wd_old + W_DIFF;}
  if(wd < (wd_old - W_DIFF)){wd = wd_old - W_DIFF;}
  wd_old = wd;*/
  //std::cout<<"wd: "<<wd<<" old: "<<wd_old<<std::endl;
  
  double delta_d = 1/std::sin(EPSILON)*std::atan(L_B*wd/vd);
  
  // Confine the steering angle difference to some designated value  
  /*static double vd_old = 0;
  if(vd > (vd_old + V_DIFF)){vd = vd_old + V_DIFF;}
  if(vd < (vd_old - V_DIFF)){vd = vd_old - V_DIFF;}
  vd_old = vd;*/
  
  
  // Compute compensation tilt angle theta_comp (consider accelleration)
  double theta_comp = -(L_A*std::sin(EPSILON)*vd)/(L_B*GRAVITY)*(vd/L_A+(Ka*(vd-v0)))*wd;
  static double theta_comp_old = 0;
  if(theta_comp > (theta_comp_old + THETA_COMP_DIFF)){theta_comp = theta_comp_old + THETA_COMP_DIFF;}
  if(theta_comp < (theta_comp_old - THETA_COMP_DIFF)){theta_comp = theta_comp_old - THETA_COMP_DIFF;}
  theta_comp_old = theta_comp;
  
  // std::cout<<"vd: "<<vd<<" wd: "<<wd<<" delta_d: "<<delta_d<< " comp: "<<theta_comp<<" theta: "<<theta<<" delta: "<<delta<<" est a: "<<Ka*(vd-v0)<<std::endl;
  
  // Perform PD control (feedback control)
  // double delta_cmd = Kp * (theta - Ki * theta_comp) + Kd * theta_dot + delta_d;
  // double delta_cmd = Kp * (theta) + Kd * theta_dot + delta_d;
  double delta_cmd = Kp * (theta - theta_comp) + Kd * theta_dot;
  double vel_cmd = vd / WHEEL_R;
  if(abs(theta)>0.8){
    delta_cmd = 0;
  }
  std::cout<<"vd: "<<vd<<" wd: "<<wd<<" delta_d: "<<delta_d<< " comp: "<<theta_comp<<" theta: "<<theta<<" delta: "<<delta<<" cmd: "<<delta_cmd<<std::endl;
  
  output[0] = vel_cmd;
  output[1] = delta_cmd;
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
