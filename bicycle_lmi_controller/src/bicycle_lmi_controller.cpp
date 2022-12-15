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
#define L_B 1.023273f
// #define L_A 0.418087f
#define L_A 0.35f
//#define L_A 1.023273f
#define dT 0.01f
#define PI 3.141592653589f
#define EPSILON 70*PI/180.0f
#define DELTA_MAX 0.6f
#define DELTA_MIN -0.6f

// Global variables - Ros Param
std::string command_topic_name;
std::string imu_topic_name;
std::string joint_topic_name;
double K1, K2, K3, Ka, DELTA_DIFF, W_DIFF;
bool control_flag = false;

// Other params for callback fcn
sensor_msgs::Imu imu_data;
geometry_msgs::Twist command;
sensor_msgs::JointState joint_state;

// Predefine functions
void LMIControl(sensor_msgs::Imu imu_data, geometry_msgs::Twist command, sensor_msgs::JointState joint_state, double* orientation, double *output);
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
  control_flag == true;
}
  
int main(int argc, char** argv){
  // Setup ros node
  ros::init(argc, argv, "bicycle_lmi_controller");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  ros::Rate loop_rate(100);
  
  // Get ros params
  private_nh.getParam("command_topic_name", command_topic_name);
  private_nh.getParam("imu_topic_name", imu_topic_name);
  private_nh.getParam("joint_topic_name", joint_topic_name);
  private_nh.getParam("K1",K1);
  private_nh.getParam("K2",K2);
  private_nh.getParam("K3",K3);
  private_nh.getParam("Ka",Ka);
  private_nh.getParam("DELTA_DIFF",DELTA_DIFF);
  private_nh.getParam("W_DIFF",W_DIFF);
  
  // Define the subscriber to odometry
  ros::Subscriber joint_sub = nh.subscribe<sensor_msgs::JointState>(joint_topic_name, 10, &jointStateCallback);
  ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>(imu_topic_name, 10, &imuCallback);
  ros::Subscriber cmd_sub = nh.subscribe<geometry_msgs::Twist>(command_topic_name, 10, &commandCallback);
  
  
  
  // Define the publishers for bicycle conroller
  ros::Publisher vel_pub = nh.advertise<std_msgs::Float64>("bike/back_wheel_controller/command", 10);
  ros::Publisher delta_pub = nh.advertise<std_msgs::Float64>("bike/front_fork_controller/command", 10);
  
  // Variables
  double vehicle_orientation[3]= {0};
  double vehicle_output[3] = {0};
  

  while (ros::ok())
  {
    quaternion2RPY(imu_data, vehicle_orientation);
    std_msgs::Float64 vd;
    std_msgs::Float64 wd;
    vd.data = command.linear.x;
    wd.data = command.angular.z;
    
    if(command.linear.x * WHEEL_R > 1.5){
      LMIControl(imu_data, command, joint_state, vehicle_orientation, vehicle_output);
    
   
      vd.data = vehicle_output[0];
      wd.data = vehicle_output[1];
    }
    /*if(control_flag == true && (double)joint_state.velocity[2] * WHEEL_R > 1.5){
      LMIControl(imu_data, command, joint_state, vehicle_orientation, vehicle_output);
      vd.data = vehicle_output[0];
      wd.data = double pi = 3.14159265359;lag == true && (double)joint_state.velocity[2] * WHEEL_R < 1.5 && command.linear.x * WHEEL_R != 0){
      if(vd.data<=1.5){
        vd.data += 0.001;
        }
    }
    if(control_flag == true && command.linear.x * WHEEL_R == 0){
      vd.data = 0.0;
    }*/
    
    //std::cout<<"v: "<<vd.data<<" wd: "<<wd.data<<" theta: "<<vehicle_orientation[0]<<std::endl;
    vel_pub.publish(vd);
    delta_pub.publish(wd);    
    
    ros::spinOnce();
    loop_rate.sleep();
  }
};


void LMIControl(sensor_msgs::Imu imu_data, geometry_msgs::Twist command, sensor_msgs::JointState joint_state, double* orientation, double *output){
  // Get bicycle states
  double vd = command.linear.x * WHEEL_R;
  double wd = command.angular.z;
  double theta = orientation[0];
  double theta_dot = imu_data.angular_velocity.x;
  double a = imu_data.linear_acceleration.x;
  double v0 = (double)joint_state.velocity[2] * WHEEL_R;
  double delta = -(double)joint_state.position[1]; // CHANGED 11/28 The motor is installed oposite against the delta direction
  
  
  // Confine the wd difference to some designated value
  static double wd_old = 0;
  if(wd > (wd_old + W_DIFF)){wd = wd_old + W_DIFF;}
  if(wd < (wd_old - W_DIFF)){wd = wd_old - W_DIFF;}
  wd_old = wd;
  
  double delta_d = 1/std::sin(EPSILON)*std::atan(L_B*wd/vd);
  
  // LMI Control
  //double K[3] = {266.1011, 43.6233, -17.3143};
  double K[3] = {K1, K2, K3};
  double x[3] = {theta, theta_dot, delta};
  double x_d[3] = {(-vd*vd*std::sin(EPSILON))/(GRAVITY*L_B)*delta_d, 0, delta_d};//delta};
  double u_d = (vd/L_A)*delta_d;  // TEST downscale by alpha
  double u_bar = 0;
  double u = 0;
  
  // Compute full-state feedback with cornering
  for(int i=0; i<3; i++){
    x[i] = x[i] - x_d[i];  
  }
  for(int i=0; i<3; i++){
    u += K[i]*x[i];
  }
  u_bar = u + u_d;  
  
  // Perform LMI control (full state feedback)
  static double delta_cmd_old = 0;
  double delta_cmd = (delta_cmd_old + dT*u_bar)/(1+v0/L_A*dT+Ka*(vd-v0)*dT);
  delta_cmd_old = delta_cmd;
  // std::cout<< std::fixed << std::setprecision(3)<<"dc: "<<delta_cmd<<" dc_old "<<delta_cmd_old<<" ub "<<u_bar<<std::endl;

  // static double delta_cmd_old = 0;
  // static double u_bar_old = 0;
  // double delta_cmd = u_bar_old*dT+(1-(v0/L_A+Ka*(vd-v0))*dT)*delta_cmd_old;
  // delta_cmd_old = delta_cmd;
  // u_bar_old = u_bar;


  

  // Confine the steering command difference to some designated value
  if(delta_cmd > DELTA_MAX){
   delta_cmd = DELTA_MAX;
  }
  if(delta_cmd < DELTA_MIN){
   delta_cmd = DELTA_MIN;
  }
  
  // std::cout<<"vd: "<<vd<<" wd: "<<wd<<" theta: "<<theta<<" cmd: "<<vd*vd*std::sin(EPSILON*(pi/180))/(-GRAVITY*L_B)*delta_d<<" delta: "<<delta<<" delta_cmd: "<<delta_cmd<<" est a: "<<vd*vd*std::sin(EPSILON*(pi/180))/(-GRAVITY*L_B)*delta_d<<std::endl;
  
  std::cout<< std::fixed << std::setprecision(3)<<"phi_dot: "<<std::tan(delta*std::sin(EPSILON)/L_B*v0)*std::cos(theta)<<" wd: "<<wd<<" delta: "<<delta<<" delta_d: "<<delta_d<<" cmd: "<<delta_cmd<<" theta: "<<theta<<" t_cmd: "<<(-vd*vd*std::sin(EPSILON))/(GRAVITY*L_B)*delta_d<<" u: "<<u_bar<<std::endl;
  // std::cout<< std::fixed << std::setprecision(3)<<"x1_err: "<<x[0]<<" x2_err: "<<x[1]<<" x3_err: "<<x[2]<<" | "<<K[0]*x[0]<<" "<<K[1]*x[1]<<" "<<K[2]*x[2]<<" "<<std::endl;
  double vel_cmd = vd / WHEEL_R;
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
