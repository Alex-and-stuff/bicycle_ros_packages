#include <fstream>
#include <iostream>
#include <string.h>
#include "pcl_ros/point_cloud.h"
#include <algorithm>
#include <iterator>
#include <Eigen/Dense>

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

#include "kf_tracker/featureDetection.h"
#include "kf_tracker/CKalmanFilter.h"
#include "opencv2/video/tracking.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>


#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>

int clusterNum = 0;
int filterNum = 0;
bool firstIter = true;
ros::Publisher markerPub;
std::vector<cv::KalmanFilter> kfVector;
std::vector<int> clusterFilterID;
std::vector<geometry_msgs::Point> KFpredictions;

// calculate euclidean distance of two points
double euclidean_distance(geometry_msgs::Point &p1, geometry_msgs::Point &p2) {
  std::cout << "p1: ("<<p1.x<<","<<p1.y<<") p2: ("<<p2.x<<","<<p2.y<<")"<<std::endl;
  return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) +
         (p1.z - p2.z) * (p1.z - p2.z);
}

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

void removeRow(Eigen::MatrixXd& matrix, unsigned int rowToRemove)
{
    unsigned int numRows = matrix.rows()-1;
    unsigned int numCols = matrix.cols();

    if( rowToRemove < numRows )
        matrix.block(rowToRemove,0,numRows-rowToRemove,numCols) = matrix.bottomRows(numRows-rowToRemove);

    matrix.conservativeResize(numRows,numCols);
}

void removeColumn(Eigen::MatrixXd& matrix, unsigned int colToRemove)
{
    unsigned int numRows = matrix.rows();
    unsigned int numCols = matrix.cols()-1;

    if( colToRemove < numCols )
        matrix.block(0,colToRemove,numRows,numCols-colToRemove) = matrix.rightCols(numCols-colToRemove);

    matrix.conservativeResize(numRows,numCols);
}

void kfPredict(){
    /* Perform prediction for each kf filter and store results to vector<kfVector> */
    std::vector<cv::Mat> pred;

    for(int i = 0; i < filterNum; i ++){
        cv::Mat kfPred = kfVector[i].predict();
        std::cout<<"kfPred: ("<<kfPred.at<float>(0)<<","<<kfPred.at<float>(1)<<") ";
        pred.push_back(kfPred);
    }
    std::cout<<std::endl;


    /* Store state prediction coordinates to the vector<KFpredictions> */
    KFpredictions.clear();
    for (auto it = pred.begin(); it != pred.end(); it++) {
        geometry_msgs::Point pt;
        pt.x = (*it).at<float>(0);
        pt.y = (*it).at<float>(1);
        pt.z = (*it).at<float>(2);
        std::cout<<"kf pred: ("<<pt.x<<","<<pt.y<<") ";
        KFpredictions.push_back(pt);
    }
    std::cout<<std::endl;
    std::cout<<"number of predictions: "<<KFpredictions.size()<<std::endl;
}

void kfUpdate(std::vector<geometry_msgs::Point> &clusterCenters){
    publishMarkers(KFpredictions);

    for(int i = 0; i < filterNum; i ++){
        float meas[2] = {0};
        meas[0] = clusterCenters[i].x;
        meas[1] = clusterCenters[i].y;
        cv::Mat measMat = cv::Mat(2, 1, CV_32F, meas);
        cv::Mat estimate = kfVector[clusterFilterID[i]].correct(measMat);
        std::cout<<"kf correct: ("<<kfVector[clusterFilterID[i]].statePost.at<float>(0)<<" "<<kfVector[clusterFilterID[i]].statePost.at<float>(1)<<")"<<std::endl;
    }
}


void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input){
    
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
    clusterNum = 0;
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

    /* Tracking of clusters */
    float dvx = 0.01f; // 1.0
    float dvy = 0.01f; // 1.0
    float dx = 1.0f;
    float dy = 1.0f;
    float sigmaP = 0.01;
    float sigmaQ = 0.1;
    int stateDim = 4; // [x,y,v_x,v_y]//,w,h]
    int measDim = 2;  // [z_x,z_y,z_w,z_h]
    int ctrlDim = 0;


    if(firstIter){
        /* Build a cooresponding filter for each filter */
        for(int i = 0; i < clusterNum ; i ++){
            cv::KalmanFilter kfFilter(stateDim, measDim, ctrlDim, CV_32F);
            kfFilter.transitionMatrix = (Mat_<float>(4, 4) << 
                                         dx, 0,  1,   0, 
                                         0,  dy, 0,   1, 
                                         0,  0,  dvx, 0, 
                                         0,  0,  0,   dvy);
            cv::setIdentity(kfFilter.measurementMatrix);
            cv::setIdentity(kfFilter.processNoiseCov, Scalar::all(sigmaP));
            cv::setIdentity(kfFilter.measurementNoiseCov, cv::Scalar(sigmaQ));
            kfFilter.statePost.at<float>(0) = clusterCenters[i].x;
            kfFilter.statePost.at<float>(1) = clusterCenters[i].y;
            kfFilter.statePost.at<float>(2) = 0;
            kfFilter.statePost.at<float>(3) = 0;
            kfVector.push_back(kfFilter);
            filterNum ++;
            KFpredictions = clusterCenters;
        }
        firstIter = false;    
    }
    else{
        /* If new cluster matches to old filter, then keep the old filter
         * Else, a new filter must be built. */
        int addFilters = clusterNum - filterNum;
        if(clusterNum > filterNum){
            /* add new filter */
            for(int i = 0; i < addFilters ; i ++){
                cv::KalmanFilter kfFilter(stateDim, measDim, ctrlDim, CV_32F);
                kfFilter.transitionMatrix = (Mat_<float>(4, 4) << 
                                            dx, 0,  1,   0, 
                                            0,  dy, 0,   1, 
                                            0,  0,  dvx, 0, 
                                            0,  0,  0,   dvy);
                cv::setIdentity(kfFilter.measurementMatrix);
                cv::setIdentity(kfFilter.processNoiseCov, Scalar::all(sigmaP));
                cv::setIdentity(kfFilter.measurementNoiseCov, cv::Scalar(sigmaQ));
                kfFilter.statePost.at<float>(0) = 0;
                kfFilter.statePost.at<float>(1) = -500;
                kfFilter.statePost.at<float>(2) = 0;
                kfFilter.statePost.at<float>(3) = 0;
                kfVector.push_back(kfFilter);
                filterNum ++;
                std::cout<<"add filter"<<filterNum-1<<std::endl;
            }
        }
        std::cout<<"c/f num1: "<<clusterNum<<" "<<filterNum<<std::endl;
        kfPredict();
        Eigen::MatrixXf distMat(filterNum, clusterNum); // <#col, #row>
        for(int nFil = 0; nFil < filterNum; nFil ++){
            for(int nClus = 0; nClus < clusterNum; nClus ++){
                distMat(nFil, nClus) = euclidean_distance(KFpredictions[nFil],clusterCenters[nClus]);
            }
        }
        Eigen::Index maxRow, maxCol; // Correspond to filter/cluster
        float maxDist = distMat.maxCoeff(&maxRow, &maxCol);
        clusterFilterID.clear();
        clusterFilterID.resize(clusterNum);
        int filterUsed[filterNum] = {false};
        for(int i = 0; i < clusterNum; i ++){
            Eigen::Index minRow, minCol; // Correspond to filter/cluster
            float minDist = distMat.minCoeff(&minRow, &minCol);
            clusterFilterID[int(minCol)] = int(minRow);
            filterUsed[clusterFilterID[int(minCol)]] = true;
            distMat.row(int(minRow)) = Eigen::MatrixXf::Constant(filterNum, 1, maxDist);
            distMat.col(int(minCol)) = Eigen::MatrixXf::Constant(1, clusterNum, maxDist);
            if(addFilters>0 && i > clusterNum - addFilters){
                kfVector[int(minRow)].statePost.at<float>(0) = clusterCenters[int(minCol)].x;
                kfVector[int(minRow)].statePost.at<float>(1) = clusterCenters[int(minCol)].y;
            }
        }
        // if(clusterNum < filterNum){
        //     /* kill un-assocoated filters */
        //     for(int i = 0; i < filterNum; i ++){
        //         if(filterUsed[i] == false){
        //             kfVector.erase(kfVector.begin()+i);
        //             std::cout<<"erase filter"<<i<<std::endl;
        //         }
        //     }
        // }

        filterNum = kfVector.size();
        if(filterNum != clusterNum){
            std::cout<<"Error! filterNum != clusterNum"<<std::endl;
            std::cout<<"c/f num: "<<clusterNum<<" "<<filterNum<<std::endl;
        }
        kfUpdate(clusterCenters);

    }

    // publishMarkers(clusterCenters);
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