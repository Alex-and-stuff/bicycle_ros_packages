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
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/conversions.h>

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
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>

#include <chrono>

#include <pcl/filters/passthrough.h>

/* Use RGB-d camera sim or use mmwave radar */
// #define USERGBD


#define MAXFILTERS      5 
#define DECAYUPBOUND    20
#define DECAYLOWBOUND   -10

#ifdef USERGBD
#define CLUSTTOL        0.6f
#define MINCLUSTSIZE    8
#define MAXCLUSTSIZE    600000
#endif

#ifndef USERGBD
#define CLUSTTOL        1.2f
#define MINCLUSTSIZE    9
#define MAXCLUSTSIZE    100
#endif

#define INCREASERATE    0.7f
#define DECREASERATE    0.2f
#define SPAWNVAL        8
#define dvx             0.01f  // 1.0
#define dvy             0.01f  // 1.0
#define dx              1.0f
#define dy              1.0f
#define sigmaP          0.05
#define sigmaQ          0.2
#define stateDim        4  // [x,y,v_x,v_y]//,w,h]
#define measDim         2  // [z_x,z_y,z_w,z_h]
#define ctrlDim         0

#define FILTER_RAD      1.0f
#define FILTER_BOT      -0.9f
#define FILTER_TOP      2.8f



int clusterNum = 0;
int filterNum = 0;
float filterDecay[MAXFILTERS] = {0};
bool firstIter = true;
ros::Publisher markerPub;
ros::Publisher posArrayPub;
std::vector<cv::KalmanFilter> kfVector;
std::vector<int> clusterFilterID;
std::vector<geometry_msgs::Point> KFpredictions;
float obstacleSize = 1.3;
int counter = 0;
pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ori(new pcl::PointCloud<pcl::PointXYZ>);
bool DATA_FLAG = false;


/* Calculate euclidean distance of two points */
double euclidean_distance(geometry_msgs::Point &p1, geometry_msgs::Point &p2) {
    return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) +
            (p1.z - p2.z) * (p1.z - p2.z);
}

void publishPoseArray(std::vector<geometry_msgs::Point> &kfPredictions){
    geometry_msgs::PoseArray poseArray;
    poseArray.header.stamp = ros::Time::now(); // timestamp of creation of the msg
    // poseArray.header.frame_id = "/base_link"; // frame id in which the array is published
    poseArray.header.frame_id = "/ti_mmwave_0"; // frame id in which the array is published
    
    geometry_msgs::Pose p; // one pose to put in the array
   
    for (int i = 0; i < kfPredictions.size(); i++) {
        if(filterDecay[i] > 0){
            p.position.x = kfPredictions[i].x;
            p.position.y = kfPredictions[i].y;
            p.position.z = obstacleSize* filterDecay[i]/DECAYUPBOUND; // Store size of obstacle in position.z

            poseArray.poses.push_back(p);
        }
    } 
    posArrayPub.publish(poseArray);

}

void publishMarkers(std::vector<geometry_msgs::Point> &kfPredictions){
    
  visualization_msgs::MarkerArray clusterMarkers;
  static int clusterNumOld = 0;

  int publishedMarkers = 0;
  static int publishedMarkersOld = 0;
  int markerID[MAXFILTERS] = {0};

  for (int i = 0; i < kfPredictions.size(); i++) {
    if(filterDecay[i] > 0){
        publishedMarkers ++;
        markerID[i] = 1;
        visualization_msgs::Marker m;
        m.id = i;
        m.type = visualization_msgs::Marker::CYLINDER;
        //m.header.frame_id = "map";
        // m.header.frame_id = "/mmwave_base_link";
        // m.header.frame_id = "/base_link";
        m.header.frame_id = "/ti_mmwave_0";

        /* Resize the marker according to its decay rate */
        m.scale.x = obstacleSize*filterDecay[i]/DECAYUPBOUND;
        m.scale.y = obstacleSize*filterDecay[i]/DECAYUPBOUND;
        // m.scale.x = kfPredictions[i].z;
        // m.scale.y = kfPredictions[i].z;
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

        m.pose.position.x = kfPredictions[i].x;
        m.pose.position.y = kfPredictions[i].y;
        m.pose.position.z = 0;

        clusterMarkers.markers.push_back(m);
    }
  }

  /* Delete the unused markers if the number of former markers 
   * are less than the current number of markers. This prevents
   * ghost markers showing up in Rviz */
    for(int i = 0; i < MAXFILTERS; i++){
        if(markerID[i] == 0){
            visualization_msgs::Marker m;
            m.id = i;
            m.action = visualization_msgs::Marker::DELETE;
            clusterMarkers.markers.push_back(m);
        }
    }
  
  clusterNumOld = clusterNum;
  publishedMarkersOld = publishedMarkers;

//   std::cout<<"Number of published markers: "<<publishedMarkers<<std::endl;
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
        // std::cout<<"kfPred: ("<<kfPred.at<float>(0)<<","<<kfPred.at<float>(1)<<") "<<std::endl;
        pred.push_back(kfPred);
    }


    /* Store state prediction coordinates to the vector<KFpredictions> */
    KFpredictions.clear();
    for (auto it = pred.begin(); it != pred.end(); it++) {
        geometry_msgs::Point pt;
        pt.x = (*it).at<float>(0);
        pt.y = (*it).at<float>(1);
        pt.z = (*it).at<float>(2);
        KFpredictions.push_back(pt);
    }
    // std::cout<<"number of predictions: "<<KFpredictions.size()<<std::endl;
}

void kfUpdate(std::vector<geometry_msgs::Point> &clusterCenters){
    /* Publish the kalman filter predictions */
    publishMarkers(KFpredictions);
    publishPoseArray(KFpredictions);

    /* Update the associated filters by checking the clusterFilterID
     * Use filterDecay[] as a decay rate for the filters, if a specific
     * filter is not updated over a span of DECAYRATE, we don't publish 
     * the filter as an obstacle */
    bool filtersUsed[MAXFILTERS] = {false};
    for(int i = 0; i < filterNum; i ++){
        if(clusterFilterID[i]!=-1){
            float meas[2] = {float(clusterCenters[i].x), float(clusterCenters[i].y)};
            cv::Mat measMat = cv::Mat(2, 1, CV_32F, meas);
            cv::Mat estimate = kfVector[clusterFilterID[i]].correct(measMat);

            if(filterDecay[clusterFilterID[i]] >= 0 && filterDecay[clusterFilterID[i]] < DECAYUPBOUND){
                filterDecay[clusterFilterID[i]]+= INCREASERATE;
            }
            if(filterDecay[clusterFilterID[i]] < 0){
                filterDecay[clusterFilterID[i]]= SPAWNVAL;
            }

            filtersUsed[clusterFilterID[i]] = true;
            // std::cout<<"kf correct: ("<<kfVector[clusterFilterID[i]].statePost.at<float>(0)
            // <<" "<<kfVector[clusterFilterID[i]].statePost.at<float>(1)<<")"<<std::endl;
        }
    }
    /* Decay filters that are not associated to clusters */
    for(int i = 0; i < filterNum; i ++){
        if(filtersUsed[i] == false){
            // std::cout<<"decay!"<<clusterFilterID[i]<<std::endl;
            if(filterDecay[i] > DECAYLOWBOUND){
                filterDecay[i] -= DECREASERATE;
            }
        }
    }
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input){
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_(new pcl::PointCloud<pcl::PointXYZ>);
    
#ifndef USERGBD
    pcl::fromROSMsg(*input, *input_cloud_ori);
#endif
#ifdef USERGBD
    pcl::fromROSMsg(*input, *input_cloud_);


    pcl::VoxelGrid<pcl::PointXYZ> sor;
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_filter(new pcl::PointCloud<pcl::PointXYZ>);
    sor.setInputCloud (input_cloud_);
    sor.setLeafSize (0.03f, 0.03f, 0.03f);
    sor.filter (*input_cloud_ori);
    
    std::cout <<"before filter: "<<input_cloud_->width * input_cloud_->height << " data points: (" << pcl::getFieldsList(*input_cloud_)<<")"<<std::endl;
    // std::cout <<"after filter: "<<input_cloud_filter->width * input_cloud_filter->height << " data points: (" << pcl::getFieldsList(*input_cloud_filter)<<")"<<std::endl;
    std::cout <<"after filter: "<<input_cloud_ori->width * input_cloud_ori->height << " data points: (" << pcl::getFieldsList(*input_cloud_ori)<<")"<<std::endl;



    if(input_cloud_ori->width * input_cloud_ori->height == 0){
        publishMarkers(KFpredictions);
        publishPoseArray(KFpredictions);
        for(int i = 0; i < filterNum; i ++){
            if(filterDecay[i] > DECAYLOWBOUND){
                filterDecay[i] -= DECREASERATE;
            }
        }
        return;
    }
#endif
    DATA_FLAG = true;
}


void object_tracking(){
    // std::cout<<"SIZE: "<<input->data.size()<<std::endl;
    // if(input->data.size()==0){
    //     return;
    // }
    auto t_start = std::chrono::steady_clock::now();
    
    /* Process the point cloud */
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);


    /* Creating the KdTree from input point cloud */
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    
    
    /* Filter out all point clouds outside of range with z: -5~3m*/
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(input_cloud_ori);
    #ifdef USERGBD
    pass.setFilterFieldName ("y");
    #endif
    #ifndef USERGBD
    pass.setFilterFieldName ("z");
    #endif
    pass.setFilterLimits (FILTER_BOT, FILTER_TOP);
    //pass.setNegative (true);
    pass.filter (*input_cloud);

    /* flatten the 3d point cloud to 2d */
    // @todo might want to add a option to choose if the user wants 3D or 2D results
    // for(int nIndex = 0; nIndex < input_cloud->points.size(); nIndex++)
    // {
    //     input_cloud->points[nIndex].z = 0;
    // }
    std::vector<pcl::PointIndices>::const_iterator it;
    std::vector<int>::const_iterator pit;
    std::vector<pcl::PointCloud<pcl::PointXYZ>> cluster_vec; 
    std::vector<geometry_msgs::Point> clusterCenters;
    
    if(input_cloud->size() == 0){
        // std::cout<<"EMPTY CLOUD!!!!!!!!!!!!!!!!!"<<std::endl;
        clusterNum = 0;
    }else{
        /* Perform clustering extraction */
        tree->setInputCloud(input_cloud);
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(CLUSTTOL);
        ec.setMinClusterSize(MINCLUSTSIZE);
        ec.setMaxClusterSize(MAXCLUSTSIZE);
        ec.setSearchMethod(tree);
        ec.setInputCloud(input_cloud);

        /* Extract the clusters out of pc and save indices in cluster_indices.*/
        ec.extract(cluster_indices);

        /* Iterate through the points in each cluster to find the centroid of the cluster */
        clusterNum = 0;
        for (it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
            clusterNum ++;
            pcl::PointCloud<pcl::PointXYZ> cloud_cluster;
            float x = 0.0;
            float y = 0.0;
            int numPts = 0;
            
            for (pit = it->indices.begin(); pit != it->indices.end(); pit++) {
                cloud_cluster.push_back(input_cloud->points[*pit]);
                #ifndef USERGBD
                x += input_cloud->points[*pit].x;  // For mmwave
                y += input_cloud->points[*pit].y;
                #endif
                
                #ifdef USERGBD
                x += input_cloud->points[*pit].x;  // TEST!!
                y += input_cloud->points[*pit].z;
                #endif
                numPts++;
            }

            // Eigen::Vector4d centriod;
            // pcl::compute3DCentroid(cloud_cluster, centriod);
            // pcl::compute3DCentroid()

            geometry_msgs::Point center;
            center.x = x / numPts;
            center.y = y / numPts;
            center.z = 0.0; 

            cluster_vec.push_back(cloud_cluster);

            // Eigen::Vector4f pivot(center.x,center.y,0,0);
            // Eigen::Vector4f maxPoint;
            // pcl::getMaxDistance(cloud_cluster, pivot, maxPoint);
            
            // /* Calculate the raduis of the cluster (max point from the center pivot point) */
            // float clusterRadius = sqrtf((center.x-maxPoint(0,0))*(center.x-maxPoint(0,0))
            //                     +(center.y-maxPoint(1,0))*(center.y-maxPoint(1,0)));

            // /* Use center.z to store the cluster radius */
            // center.z = 2*clusterRadius;

            /* Filter clusters in a certain radius too close to the sensor */
            #ifndef USERGBD
            if(center.x*center.x+center.y*center.y < FILTER_RAD*FILTER_RAD){ 
            clusterNum --;
            }else{
            clusterCenters.push_back(center);
            }
            #endif
            #ifdef USERGBD
            clusterCenters.push_back(center);
            #endif
        }
    }


    if(firstIter){
        /* 1. If it is the first iteration, setup the kalman filters
         * 2. According to the number of clusters, set the post state of the filters
         * 3. set unused filters (those that do not associate to clusters) to (-5,0)
         *  */
        for(int i = 0; i < MAXFILTERS ; i ++){
            cv::KalmanFilter kfFilter(stateDim, measDim, ctrlDim, CV_32F);
            kfFilter.transitionMatrix = (Mat_<float>(4, 4) << 
                                         dx, 0,  1,   0, 
                                         0,  dy, 0,   1, 
                                         0,  0,  dvx, 0, 
                                         0,  0,  0,   dvy);
            cv::setIdentity(kfFilter.measurementMatrix);
            cv::setIdentity(kfFilter.processNoiseCov, Scalar::all(sigmaP));
            cv::setIdentity(kfFilter.measurementNoiseCov, cv::Scalar(sigmaQ));
            if(i<clusterNum){
                kfFilter.statePost.at<float>(0) = clusterCenters[i].x;
                kfFilter.statePost.at<float>(1) = clusterCenters[i].y;
            }
            else{
                kfFilter.statePost.at<float>(0) = -5;
                kfFilter.statePost.at<float>(1) = 0;
            }
            kfFilter.statePost.at<float>(2) = 0;
            kfFilter.statePost.at<float>(3) = 0;
            kfVector.push_back(kfFilter);
            filterNum ++;
            KFpredictions = clusterCenters;
        }
        firstIter = false;    
    }
    else{
        /* 1. Perform the prediction process to all kalman filters
         * 2. Store the predicted state to vector<KFpredictions>
         * 3. If clusters are detected, associate them with filters
         * 4. Perform the update process to the associated kalman filters */
        kfPredict();

        if(clusterNum > 0 && DATA_FLAG == true){
            /* Store euclidean distances from all clusters to filters in mat<distMat> */
            Eigen::MatrixXf distMat(filterNum, clusterNum); // <#col, #row>
            for(int nFil = 0; nFil < filterNum; nFil ++){
                for(int nClus = 0; nClus < clusterNum; nClus ++){
                    distMat(nFil, nClus) = euclidean_distance(KFpredictions[nFil],clusterCenters[nClus]);
                }
            }

            /* Associate clusters to filters according to its distance (min distance between the 2) 
             * Ensure 1-to-1 correspondence by crossing out the row/col of associated cluster/filters */
            Eigen::Index maxRow, maxCol; // Corresponds to filter/cluster
            float maxDist = distMat.maxCoeff(&maxRow, &maxCol);
            clusterFilterID.clear();
            for(int i = 0; i < filterNum; i ++){
                clusterFilterID.push_back(-1);
            }
            int filterUsed[filterNum] = {false};
           
            for(int i = 0; i < clusterNum; i ++){
                Eigen::Index minRow, minCol; // Correspond to filter/cluster
                float minDist = distMat.minCoeff(&minRow, &minCol);
                clusterFilterID[int(minCol)] = int(minRow);
                filterUsed[clusterFilterID[int(minCol)]] = true;
                // distMat.row(int(minRow)) = Eigen::MatrixXf::Constant(filterNum, 1, maxDist);
                // distMat.col(int(minCol)) = Eigen::MatrixXf::Constant(1, clusterNum, maxDist);
                for(int i = 0; i < filterNum; i ++){
                    distMat(i, int(minCol)) = maxDist;
                }
                for(int i = 0; i < clusterNum; i ++){
                    distMat(int(minRow), i) = maxDist;
                }
            }
            
            /* Update the associated kalman filters with the cluster measurements (centroid) */
            kfUpdate(clusterCenters);
            // DATA_FLAG = false;
        }
        else{
            if(DATA_FLAG == true){
                publishMarkers(KFpredictions);
                publishPoseArray(KFpredictions);
                for(int i = 0; i < filterNum; i ++){
                    if(filterDecay[i] > DECAYLOWBOUND){
                        filterDecay[i] -= DECREASERATE;
                    }
                }
            }
        }
    }
}



int main(int argc, char **argv) {
    // ROS init
    ros::init(argc, argv, "kf_tracker");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::Rate loop_rate(50); 

    // Create a ROS subscriber for the input point cloud
    #ifndef USERGBD
    ROS_INFO("==== Using mmwave radar pointcloud! ====");
    ros::Subscriber sub = nh.subscribe("ti_mmwave/radar_scan_pcl_0", 1, cloud_cb);
    #endif
    #ifdef USERGBD
    ROS_INFO("==== Using RGB-d camera pointcloud! ====");
    ros::Subscriber sub = nh.subscribe("/bike/camera/depth/points", 1, cloud_cb);
    #endif

    markerPub = nh.advertise<visualization_msgs::MarkerArray>("/viz", 1);
    posArrayPub = nh.advertise<geometry_msgs::PoseArray>("/obs_pos", 1);      

    while(ros::ok()){
        object_tracking();
        ros::spinOnce();
        loop_rate.sleep();
    }
    // std::cout<<"pub"<<std::endl;
    // ros::spin();
}