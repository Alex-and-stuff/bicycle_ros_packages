#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

std::string odometry_topic_name;
std::string frame_id;
std::string child_frame_id;

void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg){
  //transform from the base_link to odom frame
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z) );
  tf::Quaternion q(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);
  
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_id));
  }
  
int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_tf");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  ros::Rate rate(200);
  
  // Get ros params
  private_nh.getParam("odometry_topic_name", odometry_topic_name);
  private_nh.getParam("frame_id", frame_id);
  private_nh.getParam("child_frame_id", child_frame_id);
  
  // Define the subscriber to odometry
  ros::Subscriber sub = nh.subscribe<nav_msgs::Odometry>(odometry_topic_name, 10, &odometryCallback);
  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
};
