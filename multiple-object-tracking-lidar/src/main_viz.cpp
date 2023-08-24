#include <fstream>
#include <iostream>
#include <string.h>
#include "pcl_ros/point_cloud.h"
#include <algorithm>

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/geometry.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>

int clusterNum;
ros::Publisher markerPub;

void publishMarkers(std::vector<geometry_msgs::Point> &clusterCenters){
  visualization_msgs::MarkerArray clusterMarkers;
  static int clusterNumOld = 0;
  for (int i = 0; i < clusterNum; i++) {
    visualization_msgs::Marker m;
    m.id = i;
    m.type = visualization_msgs::Marker::CYLINDER;
    //m.header.frame_id = "map";
    // m.header.frame_id = "/mmwave_base_link";
    m.header.frame_id = "/base_link";
    m.scale.x = 0.3;
    m.scale.y = 0.3;
    // m.scale.x = clusterCenters[i].z;
    // m.scale.y = clusterCenters[i].z;
    m.scale.z = 0.3;
    m.action = visualization_msgs::Marker::ADD;
    m.color.a = 0.5;
    m.color.r = 0;
    m.color.g = 10;
    m.color.b = 240;
    m.pose.orientation.x = 0.0;
    m.pose.orientation.y = 0.0;
    m.pose.orientation.z = 0.0;
    m.pose.orientation.w = 1.0;

    m.pose.position.x = clusterCenters[i].x;
    m.pose.position.y = clusterCenters[i].y;
    m.pose.position.z = 0;

    clusterMarkers.markers.push_back(m);
  }

  /* Delete the unused markers if the number of former markers 
   * are less than the current number of markers. This prevents
   * ghost markers showing up in Rviz */
  if(clusterNum < clusterNumOld){
    for(int i = clusterNum; i < clusterNumOld; i++){
      visualization_msgs::Marker m;
      m.id = i;
      m.action = visualization_msgs::Marker::DELETE;
      clusterMarkers.markers.push_back(m);
    }
  }
  clusterNumOld = clusterNum;
  std::cout<<"Number of published markers: "<<clusterMarkers.markers.size()<<std::endl;
  markerPub.publish(clusterMarkers);
}



void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input){
    clusterNum = 0;
    /* Process the point cloud */
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    /* Creating the KdTree from input point cloud */
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    pcl::fromROSMsg(*input, *input_cloud);

    /* flatten the 3d point cloud to 2d */
    // @todo might want to add a option to choose if the user wants 3D or 2D results
    for(int nIndex = 0; nIndex < input_cloud->points.size(); nIndex++)
    {
        input_cloud->points[nIndex].z = 0;
    }

    /* Perform clustering extraction */
    tree->setInputCloud(input_cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.5);
    ec.setMinClusterSize(8);
    ec.setMaxClusterSize(600);
    ec.setSearchMethod(tree);
    ec.setInputCloud(input_cloud);

    /* Extract the clusters out of pc and save indices in cluster_indices.*/
    ec.extract(cluster_indices);

    std::vector<pcl::PointIndices>::const_iterator it;
    std::vector<int>::const_iterator pit;
    std::vector<pcl::PointCloud<pcl::PointXYZ>> cluster_vec;  // Vector of cluster pointclouds
    std::vector<geometry_msgs::Point> clusterCenters;
    /* Iterate through the points in each cluster to find the center pos of the cluster */
    for (it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
        clusterNum ++;
        pcl::PointCloud<pcl::PointXYZ> cloud_cluster;
        float x = 0.0;
        float y = 0.0;
        int numPts = 0;
        
        for (pit = it->indices.begin(); pit != it->indices.end(); pit++) {
            cloud_cluster.push_back(input_cloud->points[*pit]);
            x += input_cloud->points[*pit].x;
            y += input_cloud->points[*pit].y;
            numPts++;
        }

        geometry_msgs::Point center;
        center.x = x / numPts;
        center.y = y / numPts;
        center.z = 0.0; 

        cluster_vec.push_back(cloud_cluster);

        Eigen::Vector4f pivot(center.x,center.y,0,0);
        Eigen::Vector4f maxPoint;
        pcl::getMaxDistance(cloud_cluster, pivot, maxPoint);
        
        float clusterRadius = sqrtf((center.x-maxPoint(0,0))*(center.x-maxPoint(0,0))
                            +(center.y-maxPoint(1,0))*(center.y-maxPoint(1,0)));

        /* Use center.z to store the cluster radius */
        center.z = 2*clusterRadius;
        std::cout<<"max point:("<<maxPoint(0,0)<<","<<maxPoint(1,0)<<") center: ("
        <<center.x<<","<<center.y<<") , Radius: "<<clusterRadius<<std::endl;

        float filterRad = 0.0;
        if(center.x*center.x+center.y+center.y < filterRad*filterRad){ // Remove cluster too near the sensor
          clusterNum --;
        }else{
          clusterCenters.push_back(center);
        }
        
    }

    std::cout<<std::endl;
    std::cout<<"number of clusters: "<<clusterNum<<std::endl;

    publishMarkers(clusterCenters);
}



int main(int argc, char **argv) {
  // ROS init
  ros::init(argc, argv, "kf_tracker");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe("ti_mmwave/radar_scan_pcl_0", 1, cloud_cb);


  markerPub = nh.advertise<visualization_msgs::MarkerArray>("viz", 1);

  ros::spin();
}