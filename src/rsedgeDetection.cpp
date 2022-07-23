#include "edge_brush/edge.h"
#include <deque>
#include <iostream>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/organized.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#define sign(x) (((x) < 0) ? -1 : ((x) > 0))
using namespace std;
typedef pcl::PointXYZ PointType;
class RsedgeDetection {
private:
  ros::NodeHandle nh_;
  ros::Subscriber subLidarCloud;
  ros::Publisher pubEdgeCloud;
  ros::Publisher pubEdgeInfo;
  ros::Publisher pubEdgeMarker;
  ros::Publisher pubClusterCloud;
  sensor_msgs::PointCloud2 clusterMsg;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOut;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2d;
  pcl::PointCloud<PointType>::Ptr edgeCloud;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree;

  float *edgeArray = new float[50];
  int *edgeIdx = new int[50];
  float resolution = 0.05;
  int queueLength = 50;
  string lidarFrame;
  std_msgs::Header cloudHeader;
  vector<PointType> edge_vec;

public:
  RsedgeDetection() {
    subLidarCloud = nh_.subscribe<sensor_msgs::PointCloud2>(
        "point_no_ground", 5, &RsedgeDetection::cloudHandler, this);

    pubEdgeInfo = nh_.advertise<edge_brush::edge>("edge_info", 1);
    pubEdgeMarker =
        nh_.advertise<visualization_msgs::MarkerArray>("edge_marker", 1);
    pubClusterCloud =
        nh_.advertise<sensor_msgs::PointCloud2>("cloud_cluster", 1);

    cloudIn.reset(new pcl::PointCloud<pcl::PointXYZI>());
    cloudOut.reset(new pcl::PointCloud<pcl::PointXYZI>());
    edgeCloud.reset(new pcl::PointCloud<PointType>());
    cloud2d.reset(new pcl::PointCloud<pcl::PointXYZ>());
    tree.reset(new pcl::search::KdTree<pcl::PointXYZ>());
    resetParameters();
  }

  ~RsedgeDetection() {}

  void cloudHandler(const sensor_msgs::PointCloud2ConstPtr &lidarCloudMsg) {
    lidarFrame = lidarCloudMsg->header.frame_id;
    cloudHeader = lidarCloudMsg->header;

    pcl::fromROSMsg(*lidarCloudMsg, *cloudIn);
    float x;
    float y;
    float abs_y;
    int idx;

    if (cloudIn->points.size() < 30)
      return;
    // cluster the edge
    pcl::copyPointCloud(*cloudIn, *cloud2d);

    for (int i = 0; i < cloud2d->points.size(); i++) {
      cloud2d->points[i].z = 0;
    }
    tree->setInputCloud(cloud2d);
    vector<pcl::PointIndices> local_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclid;
    euclid.setInputCloud(cloud2d);
    euclid.setClusterTolerance(0.1);
    euclid.setMaxClusterSize(10000);
    euclid.setMinClusterSize(1);
    euclid.setSearchMethod(tree);
    euclid.extract(local_indices);
    if (local_indices.size() == 0)
      return;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCluster(
        new pcl::PointCloud<pcl::PointXYZ>);

    for (auto pit = local_indices[0].indices.begin();
         pit != local_indices[0].indices.end(); ++pit) {
      pcl::PointXYZ p;
      p.x = cloudIn->points[*pit].x;
      p.y = cloudIn->points[*pit].y;
      p.z = cloudIn->points[*pit].z;
      cloudCluster->push_back(p);
    }

    publishCloud(pubClusterCloud, cloudCluster, cloudHeader.stamp,
                 cloudHeader.frame_id);

    for (size_t i = 0; i < cloudCluster->points.size(); i++) {
      x = cloudCluster->points[i].x;
      y = cloudCluster->points[i].y;
      idx = (int)(x / resolution);
      if (idx >= 50)
        continue;
      abs_y = abs(y);
      if (abs_y < edgeArray[idx]) {
        edgeArray[idx] = y;
        edgeIdx[idx] = i;
      }
    }

    for (size_t i = 0; i < queueLength; i++) {
      PointType thisPoint;
      int idx;
      idx = edgeIdx[i];
      if (edgeArray[i] > 10 || idx < 0)
        continue;
      // if the edge is too close ,treat it as noise point
      if (abs(cloudIn->points[idx].y) < 0.5)
        continue;
      thisPoint.x = cloudCluster->points[idx].x;
      thisPoint.y = cloudCluster->points[idx].y;
      thisPoint.z = -0.55;
      edge_vec.push_back(thisPoint);
    }
    checkEdge();

    calculatePCA();

    visualizeEdge();

    resetParameters();
  }

  float pointDistance(PointType p) {
    return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
  }

  float pointDistance(PointType p1, PointType p2) {
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) +
                (p1.z - p2.z) * (p1.z - p2.z));
  }

  void resetParameters() {
    cloudIn->points.clear();
    cloudOut->points.clear();
    edgeCloud->points.clear();
    edge_vec.clear();
    for (size_t i = 0; i < queueLength; i++) {
      edgeArray[i] = 100;
      edgeIdx[i] = -1;
    }
  }

  template <typename T>
  sensor_msgs::PointCloud2 publishCloud(const ros::Publisher &thisPub,
                                        const T &thisCloud, ros::Time thisStamp,
                                        std::string thisFrame) {
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub.getNumSubscribers() != 0)
      thisPub.publish(tempCloud);
    return tempCloud;
  }

  void checkEdge() {
    if (edge_vec.size() < 3)
      ROS_INFO("too few edge points is detected");
    return;
    for (int i = 1; i < edge_vec.size() - 1; i++) {
      if (pointDistance(edge_vec[i - 1], edge_vec[i]) > 0.1 &&
          pointDistance(edge_vec[i], edge_vec[i + 1]) > 0.1) {
        continue;
      }
      edgeCloud->emplace_back(edge_vec[i]);
    }
  }

  void calculatePCA() {
    if (edgeCloud->points.size() < 2)
      return;
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*edgeCloud, pcaCentroid);
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*edgeCloud, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(
        covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();
    Eigen::Vector3f mainDir = eigenVectorsPCA.col(2);

    double distance =
        abs((mainDir(0) * pcaCentroid(1) - mainDir(1) * pcaCentroid(0)) /
            (sqrt(mainDir(0) * mainDir(0) + mainDir(1) * mainDir(1))));
    double angelDiff = atan2(mainDir(1), mainDir(0)) * 180 / M_PI;
    edge_brush::edge ed;
    ed.distance = distance;
    ed.angleDiff = angelDiff;
    ed.Xdir = mainDir(0);
    ed.Ydir = mainDir(1);
    ed.edgePos = sign(pcaCentroid(1));
    pubEdgeInfo.publish(ed);
  }

  void visualizeEdge() {
    if (edge_vec.empty())
      return;
    visualization_msgs::MarkerArray markerArray;
    visualization_msgs::Marker markerNode;
    markerNode.header.frame_id = lidarFrame;
    markerNode.header.stamp = cloudHeader.stamp;
    markerNode.action = visualization_msgs::Marker::ADD;
    markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
    markerNode.ns = "edge_detected";
    markerNode.id = 0;
    markerNode.pose.orientation.w = 1;
    markerNode.scale.x = 0.05;
    markerNode.scale.y = 0.05;
    markerNode.scale.z = 0.05;
    markerNode.color.r = 0;
    markerNode.color.g = 0.8;
    markerNode.color.b = 1;
    markerNode.color.a = 1;

    visualization_msgs::Marker markerEdge;
    markerEdge.header.frame_id = lidarFrame;
    markerEdge.header.stamp = cloudHeader.stamp;
    markerEdge.action = visualization_msgs::Marker::ADD;
    markerEdge.type = visualization_msgs::Marker::LINE_LIST;
    markerEdge.ns = "edge_detected";
    markerEdge.id = 1;
    markerEdge.pose.orientation.w = 1;
    markerEdge.scale.x = 0.05;
    markerEdge.color.r = 0.9;
    markerEdge.color.g = 0.9;
    markerEdge.color.b = 0;
    markerEdge.color.a = 1;

    for (auto it = edge_vec.begin(); it != edge_vec.end(); ++it) {
      geometry_msgs::Point p;
      p.x = it->x;
      p.y = it->y;
      p.z = it->z;
      markerNode.points.push_back(p);
    }

    for (auto it = edge_vec.begin(); it != edge_vec.end() - 1; ++it) {
      geometry_msgs::Point p;
      p.x = it->x;
      p.y = it->y;
      p.z = it->z;
      markerEdge.points.push_back(p);
      p.x = (it + 1)->x;
      p.y = (it + 1)->y;
      p.z = (it + 1)->z;
      markerEdge.points.push_back(p);
    }

    markerArray.markers.push_back(markerNode);
    markerArray.markers.push_back(markerEdge);
    pubEdgeMarker.publish(markerArray);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "rsedgeDetection");
  RsedgeDetection rsed;
  ros::spin();
  return 0;
}
