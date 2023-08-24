#include "kf_tracker/CKalmanFilter.h"
#include "kf_tracker/featureDetection.h"
#include "opencv2/video/tracking.hpp"
#include "pcl_ros/point_cloud.h"
#include <algorithm>
#include <fstream>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <iterator>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <string.h>

#include <pcl/common/centroid.h>
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

#include <limits>
#include <utility>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;
using namespace cv;

ros::Publisher objID_pub;

// KF init
int stateDim = 4; // [x,y,v_x,v_y]//,w,h]
int measDim = 2;  // [z_x,z_y,z_w,z_h]
int ctrlDim = 0;
cv::KalmanFilter KF0(stateDim, measDim, ctrlDim, CV_32F);
cv::KalmanFilter KF1(stateDim, measDim, ctrlDim, CV_32F);
cv::KalmanFilter KF2(stateDim, measDim, ctrlDim, CV_32F);
cv::KalmanFilter KF3(stateDim, measDim, ctrlDim, CV_32F);
cv::KalmanFilter KF4(stateDim, measDim, ctrlDim, CV_32F);
cv::KalmanFilter KF5(stateDim, measDim, ctrlDim, CV_32F);

ros::Publisher pub_cluster0;
ros::Publisher pub_cluster1;
ros::Publisher pub_cluster2;
ros::Publisher pub_cluster3;
ros::Publisher pub_cluster4;
ros::Publisher pub_cluster5;

ros::Publisher markerPub;

std::vector<geometry_msgs::Point> prevClusterCenters;

cv::Mat state(stateDim, 1, CV_32F);
cv::Mat_<float> measurement(2, 1);

std::vector<int> objID; // Output of the data association using KF
                        // measurement.setTo(Scalar(0));

bool firstFrame = true;

int clusterNum = 0;
int filterNum = 0;
std::vector<cv::KalmanFilter> kfVector;

// calculate euclidean distance of two points
double euclidean_distance(geometry_msgs::Point &p1, geometry_msgs::Point &p2) {
  return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) +
              (p1.z - p2.z) * (p1.z - p2.z);
}
/*
//Count unique object IDs. just to make sure same ID has not been assigned to
two KF_Trackers. int countIDs(vector<int> v)
{
    transform(v.begin(), v.end(), v.begin(), abs); 
    // O(n) where n = distance(v.end(), v.begin()) sort(v.begin(), v.end()); 
    // Average case O(n logn), worst case O(n^2) (usually implemented as quicksort.
    // To guarantee worst case O(n log n) replace with make_heap, then sort_heap.
    // Unique will take a sorted range, and move things around to get duplicated
    // items to the back and returns an iterator to the end of the unique
section of the range auto unique_end = unique(v.begin(), v.end()); // Again n
comparisons return distance(unique_end, v.begin()); // Constant time for random
access iterators (like vector's)
}
*/

/*
objID: vector containing the IDs of the clusters that should be associated with
each KF_Tracker objID[0] corresponds to KFT0, objID[1] corresponds to KFT1 etc.

code iterates through the distMat[][], and compares the distance to find the smallest 
  - if distMat[i][j] < minEL(max val of a float) 
  - update the smaller val as minEL
  - pair the ID to the correct tracker
*/

void publishMarkers(std::vector<geometry_msgs::Point> &kf_pred){
  visualization_msgs::MarkerArray clusterMarkers;

  for (int i = 0; i < clusterNum; i++) {
    visualization_msgs::Marker m;

    m.id = i;
    m.type = visualization_msgs::Marker::CYLINDER;
    //m.header.frame_id = "map";
    // m.header.frame_id = "/mmwave_base_link";
    m.header.frame_id = "base_link";
    m.scale.x = 0.3;
    m.scale.y = 0.3;
    m.scale.z = 0.3;
    m.action = visualization_msgs::Marker::ADD;
    m.color.a = 0.3;
    m.color.r = 0;
    m.color.g = (float)30*(i+1);
    m.color.b = 255-(float)4*(i+1);

    m.pose.position.x = kf_pred[i].x;
    m.pose.position.y = kf_pred[i].y;
    m.pose.position.z = kf_pred[i].z;

    clusterMarkers.markers.push_back(m);
  }

  markerPub.publish(clusterMarkers);
}

std::pair<int, int> findIndexOfMin(std::vector<std::vector<float>> distMat) {
  cout << "findIndexOfMin cALLED\n";
  std::pair<int, int> minIndex;
  float minEl = std::numeric_limits<float>::max();
  cout << "minEl=" << minEl << "\n";
  for (int i = 0; i < distMat.size(); i++)
    for (int j = 0; j < distMat.at(0).size(); j++) {
      // if (distMat[i][j] < minEl) {
      //   minEl = distMat[i][j];
      //   minIndex = std::make_pair(i, j);
      // }
      minIndex = std::make_pair(i, std::min_element(distMat.at(0).begin(),distMat.at(0).end()) - distMat.at(0).begin());
      minEl = distMat[i][minIndex.second];
    }cluster_vec
void clusterAssociate_old(std::vector<std::vector<float>> distMat){
  //old method
  for (int clusterCount = 0; clusterCount < 6; clusterCount++) {
    // 1. Find min(distMax)==> (i,j);
    std::pair<int, int> minIndex(findIndexOfMin(distMat));
    cout << "Received minIndex=" << minIndex.first << "," << minIndex.second << "\n";
    // 2. objID[i]=clusterCenters[j]; counter++
    objID[minIndex.first] = minIndex.second;

    // 3. distMat[i,:]=10000; distMat[:,j]=10000
    distMat[minIndex.first] = std::vector<float>(6, 10000.0); // Set the row to a high number.
    for (int row = 0; row < distMat.size(); row++) // set the column to a high number
    {
      distMat[row][minIndex.second] = 10000.0;
    }
    // 4. if(counter<6) got to 1.
    cout << "clusterCount=" << clusterCount << "\n";
  }
}

struct distID{
  float dist; // distance square between filter and cluster
  int cID;    // cluster ID
  int fID;    // filter ID
}

bool compareDistVec(distID ele1, distID ele2){
  return (ele1.dist < ele2.dist);
}

bool compareDistMat(distID vec1, distID vec2){
  return (vec1[0].dist < vec2[0].dist);
}

void clusterAssociate(std::vector<geometry_msgs::Point> const &KFpredictions, 
                      std::vector<geometry_msgs::Point> const &clusterCenters, int* clusterToFilterID){
  /* Generate a vector storining the distance from a single KF to all clusters 
   * Vector is then sorted by cluster-filter distance 
   */
  std::vector<float> distVec;
  for (int nFilter = 0; nFilter < clusterNum; nFilter++) {
    for (int nCluster = 0; nCluster < clusterNum; nCluster++) {
      distID cfDist = {euclidean_distance(KFpredictions[nFilter], clusterCenters[nCluster]), nCluster, nFilter};
      distVec.push_back(cfDist);
    }
  }
  std::sort(distVec.begin(), distVec.end(), compareDistVec);

  /* Go through the distVec until the cluster-filter distance is too large */
  float DISTMARGIN = 0.3;
  int associateFilters = 0;
  int nObj = 0;
  int matSize = distMat.size();
  float objDist = distMat[nObj].at(0).dist;
  int objCID = distMat[nObj].at(0).cID;
  int objFID = distMat[nObj].at(0).fID;
  while(associateFilters <= clusterNum && objDist < DISTMARGIN && nObj < matSize){
    if(clusterToFilterID[objCID]==-1){
      clusterToFilterID[objCID] = objFID;
      associateFilters ++;
    }
    nObj++;
    objDist = distMat[nObj].at(0).dist;
    objCID = distMat[nObj].at(0).cID;
    objFID = distMat[nObj].at(0).fID;
  }
}

void KFT(std::vector<geometry_msgs::Point> &clusterCenters, int* clusterToFilterID) {

  /* Perform prediction for each kf filter and store results to vector<kfVector> */
  std::vector<cv::Mat> pred;
  std::cout<<"kfvec size: "<<kfVector.size()<<std::endl;
  // cv::KalmanFilter KF0 = kfVector.at(0);
  // KF0.predict();
  for(int i = 0; i < clusterNum; i ++){
    pred.push_back(kfVector[i].predict());
  }


  /* Store state prediction coordinates to the vector<KFpredictions> */
  std::vector<geometry_msgs::Point> KFpredictions;
  for (auto it = pred.begin(); it != pred.end(); it++) {
    geometry_msgs::Point pt;
    pt.x = (*it).at<float>(0);
    pt.y = (*it).at<float>(1);
    pt.z = (*it).at<float>(2);

    KFpredictions.push_back(pt);
  }

  filterNum = kfVector.size();
  // objID.clear();
  // objID.resize(clusterNum);

  /* Build an array to associate the clusters with the filters' ID */
  // int clusterToFilterID[clusterNum] = {0};
  // int clusterToFilterID[clusterNum]
  for(int i = 0; i < clusterNum; i ++){
    clusterToFilterID[i] = -1;
  }

  /* Associate clusters with kf filters, fillout the clusterToFilter[] */
  clusterAssociate(KFpredictions, clusterCenters, clusterToFilterID);

  /* If there are clusters that are not associated with filters, build a new filter */
  float dvx = 0.01f; // 1.0
  float dvy = 0.01f; // 1.0
  float dx = 1.0f;
  float dy = 1.0f;
  float sigmaP = 0.01;
  float sigmaQ = 0.1;
  int filterUsed[filterNum] = {0};
  for(int i = 0; i < clusterNum; i ++){
    if(clusterToFilterID[i] == -1){
      // Add new filter
      cv::KalmanFilter newFilter(stateDim, measDim, ctrlDim, CV_32F);
      kfFilter.transitionMatrix = (Mat_<float>(4, 4) << dx, 0, 1, 0, 0, dy, 0, 1, 0, 0,
                                  dvx, 0, 0, 0, 0, dvy);
      cv::setIdentity(kfFilter.measurementMatrix);
      cv::setIdentity(kfFilter.processNoiseCov, Scalar::all(sigmaP));
      cv::setIdentity(kfFilter.measurementNoiseCov, cv::Scalar(sigmaQ));
      kfFilter.statePre.at<float>(0) = clusterCentroids[i].x;
      kfFilter.statePre.at<float>(1) = clusterCentroids[i].y;
      kfFilter.statePre.at<float>(2) = 0;
      kfFilter.statePre.at<float>(3) = 0;

      kfVector.push_back(kfFilter);
    }
    else{
      // record the used filter
      filterUsed[clusterToFilterID[i]] = 1;
    }
  }

  /* If there are unassociated filters, delete them from the kfVector */
  int removals = 0;
  for(int i = 0; i < filterNum; i ++){
    if(filterUsed[i] == 0){
      // remove filter if not associated
      kfVector.erase(kfVector.begin+i-removals);
      removals ++;
    }
  }

  /*  */
  publishMarkers(KFpredictions);

  for(int i = 0; i < filterNum; i ++){
    if(clusterToFilterID[i] != 1){
      float meas[2] = {0};
      meas[0] = clusterCenters[clusterToFilterID[i]].x;
      meas[1] = clusterCenters[clusterToFilterID[i]].y;
      cv::Mat measMat = cv::Mat(2, 1, CV_32F, meas);
      kfVector[clusterToFilterID[i]].correct(measMat);
    }
  }
  // // std_msgs::Int32MultiArray obj_id;
  // // for (auto it = objID.begin(); it != objID.end(); it++)
  // //   obj_id.data.push_back(*it);
  // // // Publish the object IDs
  // // objID_pub.publish(obj_id);
  // // convert clusterCenters from geometry_msgs::Point to floats
  // std::vector<std::vector<float>> cc;
  // for (int i = 0; i < 6; i++) {
  //   vector<float> pt;
  //   pt.push_back(clusterCenters[objID[i]].x);
  //   pt.push_back(clusterCenters[objID[i]].y);
  //   pt.push_back(clusterCenters[objID[i]].z);

  //   cc.push_back(pt);
  // } 

  // float meas0[2] = {cc[0].at(0), cc[0].at(1)};
  // float meas1[2] = {cc[1].at(0), cc[1].at(1)};
  // float meas2[2] = {cc[2].at(0), cc[2].at(1)};
  // float meas3[2] = {cc[3].at(0), cc[3].at(1)};
  // float meas4[2] = {cc[4].at(0), cc[4].at(1)};
  // float meas5[2] = {cc[5].at(0), cc[5].at(1)};

  // // The update phase
  // cv::Mat meas0Mat = cv::Mat(2, 1, CV_32F, meas0);
  // cv::Mat meas1Mat = cv::Mat(2, 1, CV_32F, meas1);
  // cv::Mat meas2Mat = cv::Mat(2, 1, CV_32F, meas2);
  // cv::Mat meas3Mat = cv::Mat(2, 1, CV_32F, meas3);
  // cv::Mat meas4Mat = cv::Mat(2, 1, CV_32F, meas4);
  // cv::Mat meas5Mat = cv::Mat(2, 1, CV_32F, meas5);

  // // cout<<"meas0Mat"<<meas0Mat<<"\n";
  // if (!(meas0Mat.at<float>(0, 0) == 0.0f || meas0Mat.at<float>(1, 0) == 0.0f))
  //   Mat estimated0 = KF0.correct(meas0Mat);
  // if (!(meas1[0] == 0.0f || meas1[1] == 0.0f))
  //   Mat estimated1 = KF1.correct(meas1Mat);
  // if (!(meas2[0] == 0.0f || meas2[1] == 0.0f))
  //   Mat estimated2 = KF2.correct(meas2Mat);
  // if (!(meas3[0] == 0.0f || meas3[1] == 0.0f))
  //   Mat estimated3 = KF3.correct(meas3Mat);
  // if (!(meas4[0] == 0.0f || meas4[1] == 0.0f))
  //   Mat estimated4 = KF4.correct(meas4Mat);
  // if (!(meas5[0] == 0.0f || meas5[1] == 0.0f))
  //   Mat estimated5 = KF5.correct(meas5Mat);

}
void publish_cloud(ros::Publisher &pub,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr cluster) {
  sensor_msgs::PointCloud2::Ptr clustermsg(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*cluster, *clustermsg);
  // clustermsg->header.frame_id = "map";
  clustermsg->header.frame_id = "/mmwave_base_link";
  clustermsg->header.stamp = ros::Time::now();
  pub.publish(*clustermsg);
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input){
  
  if (firstFrame) {
    // Initialize 6 Kalman Filters; Assuming 6 max objects in the dataset.
    // Could be made generic by creating a Kalman Filter only when a new object is detected

    /* Process the point cloud */
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);

    /* Creating the KdTree from input point cloud */
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZ>);

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
    ec.setMinClusterSize(10);
    ec.setMaxClusterSize(600);
    ec.setSearchMethod(tree);
    ec.setInputCloud(input_cloud);

    /* Extract the clusters out of pc and save indices in cluster_indices.*/
    ec.extract(cluster_indices);


    std::vector<pcl::PointIndices>::const_iterator it;
    std::vector<int>::const_iterator pit;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_vec;  // Vector of cluster pointclouds
    std::vector<pcl::PointXYZ> clusterCentroids;

    /* Iterate through the points in each cluster to find the center pos of the cluster */
    for (it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(
          new pcl::PointCloud<pcl::PointXYZ>);
      float x = 0.0;
      float y = 0.0;
      int numPts = 0;
      clusterNum ++;  // Record the number of total clusters
      for (pit = it->indices.begin(); pit != it->indices.end(); pit++) {

        cloud_cluster->points.push_back(input_cloud->points[*pit]);
        x += input_cloud->points[*pit].x;
        y += input_cloud->points[*pit].y;
        numPts++;
      }
      
      /* Average the coordinates within the cluster to find its centroid */
      pcl::PointXYZ centroid;
      centroid.x = x / numPts;
      centroid.y = y / numPts;
      centroid.z = 0.0;  // @todo might want to add a option to choose if the user wants 3D or 2D results

      cluster_vec.push_back(cloud_cluster);

      /* Store the centroids to clusterCentroid vector*/
      clusterCentroids.push_back(centroid);
    }

    /* Setup the KF filter parameters */
    float dvx = 0.01f; // 1.0
    float dvy = 0.01f; // 1.0
    float dx = 1.0f;
    float dy = 1.0f;
    float sigmaP = 0.01;
    float sigmaQ = 0.1;

    /* Setup the KF filters according to the number of clusters */
    for(int i = 0; i < clusterNum; i ++){
      cv::KalmanFilter kfFilter(stateDim, measDim, ctrlDim, CV_32F);
      kfFilter.transitionMatrix = (Mat_<float>(4, 4) << dx, 0, 1, 0, 0, dy, 0, 1, 0, 0,
                                  dvx, 0, 0, 0, 0, dvy);
      cv::setIdentity(kfFilter.measurementMatrix);
      cv::setIdentity(kfFilter.processNoiseCov, Scalar::all(sigmaP));
      cv::setIdentity(kfFilter.measurementNoiseCov, cv::Scalar(sigmaQ));

      kfFilter.statePre.at<float>(0) = clusterCentroids[i].x;
      kfFilter.statePre.at<float>(1) = clusterCentroids[i].y;
      kfFilter.statePre.at<float>(2) = 0;
      kfFilter.statePre.at<float>(3) = 0;

      kfVector.push_back(kfFilter);
    }

    ROS_INFO("Number of clusters: %d",clusterNum);

    

    for (int i = 0; i < clusterNum; i ++) {
      geometry_msgs::Point pt;
      pt.x = clusterCentroids.at(i).x;
      pt.y = clusterCentroids.at(i).y;
      prevClusterCenters.push_back(pt);
    }
    
    /* Switch case to the else statment */
    firstFrame = false;


      /* The thing about seting filters without a limit is that we are not sure of when the cluster will appear/disappear,
     * meaning that some of the old filters should be removed when clusters disappear and new filters should be rebuilt
     * when new clusters emerge. 
     * Maybe we can initialize a new filter when the distance between the old and all new clusters are too far apart 
     * (utilizing the "prevClusterCenters" vector).
     * We can also compare the number of clusters to decide stuff 
     *   - clusterNumOld > clusterNum: object disappear, remove its filter
     *   - clusterNumOld < clusterNum: object appear, build new filter
     *   - clusterNumOld = clusterNum: (case 1) objects are the same, no filter are needed to be added or removed
     *   - clusterNumOld = clusterNum: (case 2) one object leaves, one object enters, one filter should be remeved, one should be built
     *   - Special case: multiple objects disappear, reappear causing the cases to be wrong...
     * The safest solution might still be removing filters when we cant find clusters close to the filters origin
     * */
  }
  else {
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);

    /* Creating the KdTree from input point cloud */
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZ>);

    pcl::fromROSMsg(*input, *input_cloud);

    /* flatten the 3d point cloud to 2d */    
    for(int nIndex = 0; nIndex < input_cloud->points.size(); nIndex++)
    {
      input_cloud->points[nIndex].z = 0;
    }

    tree->setInputCloud(input_cloud);

    /* Here we are creating a vector of PointIndices, which contains the actual
     * index information in a vector<int>. The indices of each detected cluster
     * are saved here. Cluster_indices is a vector containing one instance of
     * PointIndices for each detected cluster. Cluster_indices[0] contain all
     * indices of the first cluster in input point cloud.
     */
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.5);
    ec.setMinClusterSize(10);
    ec.setMaxClusterSize(600);
    ec.setSearchMethod(tree);
    ec.setInputCloud(input_cloud);
    /* Extract the clusters out of pc and save indices in cluster_indices.*/
    ec.extract(cluster_indices);

    /* To separate each cluster out of the vector<PointIndices> we have to
     * iterate through cluster_indices, create a new PointCloud for each
     * entry and write all points of the current cluster in the PointCloud.
     */

    std::vector<pcl::PointIndices>::const_iterator it;
    std::vector<int>::const_iterator pit;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_vec;  // Vector of cluster pointclouds
    std::vector<geometry_msgs::Point> clusterCentroids;

    /* Cluster centroids (Find centroids by averaging the position coordinates) */
    clusterNum = 0;
    for (it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
      float x = 0.0;
      float y = 0.0;
      int numPts = 0;
      clusterNum ++;
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(
          new pcl::PointCloud<pcl::PointXYZ>);
      for (pit = it->indices.begin(); pit != it->indices.end(); pit++) {

        cloud_cluster->points.push_back(input_cloud->points[*pit]);

        x += input_cloud->points[*pit].x;
        y += input_cloud->points[*pit].y;
        numPts++;
      }

      geometry_msgs::Point centroid;
      centroid.x = x / numPts;
      centroid.y = y / numPts;
      centroid.z = 0.0;

      cluster_vec.push_back(cloud_cluster);

      /* Store the centroids to clusterCentroid vector */
      clusterCentroids.push_back(centroid);
    }


    KFT(clusterCentroids);
  }
}

int main(int argc, char **argv) {
  // ROS init
  ros::init(argc, argv, "kf_tracker");
  ros::NodeHandle nh;

  // Publishers to publish the state of the objects (pos and vel)
  // objState1=nh.advertise<geometry_msgs::Twist> ("obj_1",1);

  cout << "About to setup callback\n";

  // Create a ROS subscriber for the input point cloud
  // ros::Subscriber sub = nh.subscribe("filtered_cloud", 1, cloud_cb);
  ros::Subscriber sub = nh.subscribe("ti_mmwave/radar_scan_pcl_0", 1, cloud_cb);
  // Create a ROS publisher for the output point cloud

  // pub_cluster0 = nh.advertise<sensor_msgs::PointCloud2>("cluster_0", 1);
  // pub_cluster1 = nh.advertise<sensor_msgs::PointCloud2>("cluster_1", 1);
  // pub_cluster2 = nh.advertise<sensor_msgs::PointCloud2>("cluster_2", 1);
  // pub_cluster3 = nh.advertise<sensor_msgs::PointCloud2>("cluster_3", 1);
  // pub_cluster4 = nh.advertise<sensor_msgs::PointCloud2>("cluster_4", 1);
  // pub_cluster5 = nh.advertise<sensor_msgs::PointCloud2>("cluster_5", 1);

  // Subscribe to the clustered pointclouds
  // ros::Subscriber c1=nh.subscribe("ccs",100,KFT);
  objID_pub = nh.advertise<std_msgs::Int32MultiArray>("obj_id", 1);
  /* Point cloud clustering
   */

  // cc_pos=nh.advertise<std_msgs::Float32MultiArray>("ccs",100);//clusterCenter1
  markerPub = nh.advertise<visualization_msgs::MarkerArray>("viz", 1);

  /* Point cloud clustering
   */

  ros::spin();
}
