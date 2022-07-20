#include "edge_brush/edge.h"
#include <deque>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;
typedef pcl::PointXYZ PointType;
#define sign(x) (((x) < 0) ? -1 : ((x) > 0))

struct RSPointXYZR {
  PCL_ADD_POINT4D
  std::uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(RSPointXYZR,
                                  (float, x, x)(float, y, y)(float, z,
                                                             z)(std::uint16_t,
                                                                ring, ring))
using PointXYZR = RSPointXYZR;
struct pointwithsmooth {
  PointType point;
  float range;
  int col;
  double smooth;
};

struct by_col {
  bool operator()(pointwithsmooth const &left, pointwithsmooth const &right) {
    return left.col < right.col;
  }
};

struct by_xvalue {
  bool operator()(PointType const &left, PointType const &right) {
    return left.x < right.x;
  }
};

class EdgeDetection {
private:
  ros::NodeHandle nh_;
  ros::Subscriber subLidarCloud;
  ros::Subscriber subOdomRaw;
  ros::Publisher pubTrimCloud;
  ros::Publisher pubEdgeCloud;
  ros::Publisher pubEdgeMarker;
  ros::Publisher pubEdgeInfo;

  std_msgs::Header cloudHeader;

  sensor_msgs::PointCloud2 currentCloudMsg;

  pcl::PointCloud<PointXYZR>::Ptr lidarCloudIn;
  pcl::PointCloud<PointXYZR>::Ptr trimmedCloud;
  pcl::PointCloud<PointType>::Ptr edgeCloud;

  // Topics and Frames
  string pointCloudTopic;
  string lidarFrame;
  string odomTopic;

  Eigen::Affine3f curodomAffine;
  Eigen::Affine3f lastodomAffine;
  Eigen::Affine3f increodomAffineInv;
  Eigen::Affine3f keyposeAffine;

  int ringFlag;
  int keyposeFlag;
  float edgeThreshold;
  bool firstTrans = true;
  //修改到yaml
  vector<float> min_pt = {0, -2, -1.5, 0};
  vector<float> max_pt = {1, 2, -1.1, 0};
  vector<vector<pointwithsmooth>> cloud_vec;
  vector<PointType> edge_vec;
  deque<PointType> edgeQueue;
  mutex mtx;

public:
  EdgeDetection();
  ~EdgeDetection();
  void allocateMemory();
  void resetParameters();
  void cloudHandler(const sensor_msgs::PointCloud2ConstPtr &lidarCloudMsg);
  void odomHandler(const nav_msgs::Odometry::ConstPtr &odomMsg);
  void areaFilter();
  template <typename T>
  void publishCloud(const ros::Publisher &thisPub, const T &thisCloud,
                    ros::Time thisStamp, string thisFrame);
  void projectCloud();
  void getEdgeCloud();
  void calculateSmooth(int ringId);
  void findEdgeCloud(int ringId);
  void transEdgeCloud();
  void checkEdgeCloud(pointwithsmooth &p);
  void calculatePCA();

  void visualizeEdge();
  float pointDistance(PointType p) {
    return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
  }

  float pointDistance(PointType p1, PointType p2) {
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) +
                (p1.z - p2.z) * (p1.z - p2.z));
  }
  int calculateCol(PointType p) {
    float horizonAngle = atan2(p.x, p.y) * 180 / M_PI;
    static float ang_res_x = 360.0 / float(1800);
    int columnIdn = round(horizonAngle / ang_res_x);
    return columnIdn;
  }
};
