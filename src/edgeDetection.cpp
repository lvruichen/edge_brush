#include "edge_brush/edgeDetection.h"
EdgeDetection::EdgeDetection() {
  ringFlag = 0;
  nh_.param<std::string>("edge_brush/pointCloudTopic", pointCloudTopic,
                         "lidar_points");
  nh_.param<std::string>("edge_brush/lidarFrame", lidarFrame, "velodyne");
  nh_.param<std::string>("edge_brush/odomTopic", odomTopic,
                         "odom_basefootprint");
  nh_.param<float>("edge_brush/edgeThreshold", edgeThreshold, 0.01);
  nh_.param<vector<float>>("edge_brush/trimDlimit", min_pt, {0, -2, -1.5, 0});
  nh_.param<vector<float>>("edge_brush/trimUlimit", max_pt, {1, 2, 0, 0});

  subLidarCloud = nh_.subscribe<sensor_msgs::PointCloud2>(
      pointCloudTopic, 5, &EdgeDetection::cloudHandler, this);
  subOdomRaw = nh_.subscribe<nav_msgs::Odometry>(
      odomTopic, 20, &EdgeDetection::odomHandler, this);
  pubEdgeCloud = nh_.advertise<sensor_msgs::PointCloud2>("edge_cloud", 1);
  pubTrimCloud = nh_.advertise<sensor_msgs::PointCloud2>("trim_cloud", 1);
  pubEdgeMarker =
      nh_.advertise<visualization_msgs::MarkerArray>("edge_marker", 1);
  pubEdgeInfo = nh_.advertise<edge_brush::edge>("edge_info", 1);
  allocateMemory();
}

EdgeDetection::~EdgeDetection() {}

void EdgeDetection::allocateMemory() {
  lidarCloudIn.reset(new pcl::PointCloud<PointXYZR>());
  trimmedCloud.reset(new pcl::PointCloud<PointXYZR>());
  edgeCloud.reset(new pcl::PointCloud<PointType>());
  cloud_vec.resize(32);
  resetParameters();
}

void EdgeDetection::cloudHandler(
    const sensor_msgs::PointCloud2ConstPtr &lidarCloudMsg) {
  std::lock_guard<std::mutex> lock(mtx);
  cloudHeader = lidarCloudMsg->header;
  currentCloudMsg = std::move(*lidarCloudMsg);
  pcl::moveFromROSMsg(currentCloudMsg, *lidarCloudIn);
  if (lidarCloudIn->is_dense == false) {
    ROS_WARN("Point cloud is not in dense format, please remove NaN points "
             "first!");
    ros::shutdown();
    // vector<int> mapping;
    // pcl::removeNaNFromPointCloud(*lidarCloudIn, *lidarCloudIn, mapping);
  }
  for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i) {
    if (currentCloudMsg.fields[i].name == "ring") {
      ringFlag = 1;
      break;
    }
  }
  if (ringFlag == 0) {
    ROS_WARN("Point cloud ring channel not available, please configure "
             "your point cloud data!");
    ros::shutdown();
  }
  areaFilter();
  publishCloud(pubTrimCloud, trimmedCloud, cloudHeader.stamp, lidarFrame);
  projectCloud();
  getEdgeCloud();
  transEdgeCloud();
  // cout << edgeQueue.size() << endl;
  publishCloud(pubEdgeCloud, edgeCloud, cloudHeader.stamp, lidarFrame);
  calculatePCA();
  visualizeEdge();
  resetParameters();
}

Eigen::Affine3f odom2affine(nav_msgs::Odometry odom) {
  double x, y, z, roll, pitch, yaw;
  x = odom.pose.pose.position.x;
  y = odom.pose.pose.position.y;
  z = odom.pose.pose.position.z;
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(odom.pose.pose.orientation, orientation);
  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
  return pcl::getTransformation(x, y, z, roll, pitch, yaw);
}

void EdgeDetection::odomHandler(const nav_msgs::Odometry::ConstPtr &odomMsg) {
  lock_guard<std::mutex> lock(mtx);
  static bool firstTrans = true;
  if (firstTrans) {
    curodomAffine = odom2affine(*odomMsg);
    lastodomAffine = curodomAffine;
    keyposeAffine = curodomAffine;
    increodomAffineInv = Eigen::Affine3f::Identity();
    firstTrans = false;
    keyposeFlag = 0;
    return;
  }
  curodomAffine = odom2affine(*odomMsg);
  increodomAffineInv = curodomAffine.inverse() * lastodomAffine;
  // change the queue to the current frame
  lastodomAffine = curodomAffine;
  Eigen::Affine3f increKeyPose;
  increKeyPose = keyposeAffine.inverse() * curodomAffine;
  float increDis = increKeyPose.translation().norm();
  if (increDis > 0.1) {
    keyposeAffine = curodomAffine;
    keyposeFlag = 1;
  }
}

void EdgeDetection::transEdgeCloud() {
  for (int i = 0; i < edgeQueue.size(); i++) {
    PointType thisPoint;
    thisPoint.x = increodomAffineInv(0, 0) * edgeQueue[i].x +
                  increodomAffineInv(0, 1) * edgeQueue[i].y +
                  increodomAffineInv(0, 2) * edgeQueue[i].z +
                  increodomAffineInv(0, 3);
    thisPoint.y = increodomAffineInv(1, 0) * edgeQueue[i].x +
                  increodomAffineInv(1, 1) * edgeQueue[i].y +
                  increodomAffineInv(1, 2) * edgeQueue[i].z +
                  increodomAffineInv(1, 3);
    thisPoint.z = increodomAffineInv(2, 0) * edgeQueue[i].x +
                  increodomAffineInv(2, 1) * edgeQueue[i].y +
                  increodomAffineInv(2, 2) * edgeQueue[i].z +
                  increodomAffineInv(2, 3);
    edgeQueue[i].x = thisPoint.x;
    edgeQueue[i].y = thisPoint.y;
    edgeQueue[i].z = thisPoint.z;
    sort(edgeQueue.begin(), edgeQueue.end(), by_xvalue());
  }
  while (edgeQueue.front().x < -2) {
    edgeQueue.pop_front();
  }
}

void EdgeDetection::resetParameters() {
  lidarCloudIn->clear();
  trimmedCloud->clear();
  edgeCloud->clear();
  edge_vec.clear();
  for (size_t i = 0; i < cloud_vec.size(); i++) {
    cloud_vec[i].clear();
  }
}

void EdgeDetection::areaFilter() {
  if (lidarCloudIn->empty())
    return;

  vector<int> indices;
  Eigen::Vector4f min_pt_vec(min_pt[0], min_pt[1], min_pt[2], min_pt[3]);
  Eigen::Vector4f max_pt_vec(max_pt[0], max_pt[1], max_pt[2], max_pt[3]);

  pcl::getPointsInBox(*lidarCloudIn, min_pt_vec, max_pt_vec, indices);
  for (size_t i = 0; i < indices.size(); i++) {
    trimmedCloud->points.push_back(lidarCloudIn->points[indices[i]]);
  }
}

void EdgeDetection::projectCloud() {
  int cloudSize = trimmedCloud->points.size();
  for (size_t i = 0; i < cloudSize; i++) {
    PointType thisPoint;
    thisPoint.x = trimmedCloud->points[i].x;
    thisPoint.y = trimmedCloud->points[i].y;
    thisPoint.z = trimmedCloud->points[i].z;
    int thisring = trimmedCloud->points[i].ring;
    pointwithsmooth thisPWS;
    thisPWS.point = thisPoint;
    thisPWS.col = calculateCol(thisPoint);
    thisPWS.range = pointDistance(thisPoint);
    thisPWS.smooth = 0;
    cloud_vec[thisring].push_back(thisPWS);
  }
}

void EdgeDetection::getEdgeCloud() {
  for (size_t i = 0; i < cloud_vec.size(); i++) {
    if (cloud_vec[i].empty() || cloud_vec[i].size() < 100)
      continue;
    int validRing = i;
    std::sort(cloud_vec[i].begin(), cloud_vec[i].end(), by_col());
    calculateSmooth(validRing);
    findEdgeCloud(validRing);
    if (keyposeFlag) {
      for (size_t i = 0; i < edge_vec.size(); i++) {
        edgeQueue.push_back(edge_vec[i]);
      }
      keyposeFlag = 0;
    }
  }
}

void EdgeDetection::calculateSmooth(int ringId) {
  int cloudSize = cloud_vec[ringId].size();
  for (int i = 5; i < cloudSize - 6; i++) {
    int columnDiff = std::abs(
        int(cloud_vec[ringId][i - 5].col - cloud_vec[ringId][i + 5].col));
    if (columnDiff > 15)
      continue;
    float smoothness =
        cloud_vec[ringId][i - 5].range + cloud_vec[ringId][i - 4].range +
        cloud_vec[ringId][i - 3].range + cloud_vec[ringId][i - 2].range +
        cloud_vec[ringId][i - 1].range + cloud_vec[ringId][i + 1].range +
        cloud_vec[ringId][i + 2].range + cloud_vec[ringId][i + 3].range +
        cloud_vec[ringId][i + 4].range + cloud_vec[ringId][i + 5].range -
        cloud_vec[ringId][i].range * 10;
    cloud_vec[ringId][i].smooth = smoothness * smoothness;
  }
}

void EdgeDetection::findEdgeCloud(int ringId) {
  PointType thisPoint;
  float maxSmooth = 0;
  int idx = -1;
  int cloudSize = cloud_vec[ringId].size();

  for (int i = 0; i < cloudSize; i++) {
    checkEdgeCloud(cloud_vec[ringId][i]);
    if (cloud_vec[ringId][i].smooth > maxSmooth) {
      maxSmooth = cloud_vec[ringId][i].smooth;
      idx = i;
    }
  }
  if (idx != -1) {
    thisPoint = cloud_vec[ringId][idx].point;
    edge_vec.push_back(thisPoint);

    edgeCloud->points.push_back(thisPoint);
  }
}

void EdgeDetection::checkEdgeCloud(pointwithsmooth &p) {
  if (p.smooth < edgeThreshold || (abs(p.point.y) < 0))
    p.smooth = 0;
}

template <typename T>
void EdgeDetection::publishCloud(const ros::Publisher &thisPub,
                                 const T &thisCloud, ros::Time thisStamp,
                                 string thisFrame) {
  sensor_msgs::PointCloud2 tempCloud;
  pcl::toROSMsg(*thisCloud, tempCloud);
  tempCloud.header.stamp = thisStamp;
  tempCloud.header.frame_id = thisFrame;
  thisPub.publish(tempCloud);
}

void EdgeDetection::calculatePCA() {
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

void EdgeDetection::visualizeEdge() {
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