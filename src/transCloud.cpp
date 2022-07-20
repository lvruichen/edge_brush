#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
using namespace std;

class TransCloud {
private:
  ros::NodeHandle nh_;
  ros::Subscriber subLidarCloud;
  ros::Publisher pubLidarCloud;
  string frame_id = "rslidar_sim";
  string pointCloudTopic = "/rslidar_points";
  string pubTopic = "/rslidar_points_trans";
  std_msgs::Header header;
  sensor_msgs::PointCloud2 transCloudMsg;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOut;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudTrim;
  Eigen::Affine3f transMatrix;
  vector<float> min_pt = {0, -2, -1.5, 0};
  vector<float> max_pt = {2, 2, 0, 0};

public:
  TransCloud() {
    subLidarCloud = nh_.subscribe<sensor_msgs::PointCloud2>(
        pointCloudTopic, 10, &TransCloud::cloudHandler, this);
    pubLidarCloud = nh_.advertise<sensor_msgs::PointCloud2>(pubTopic, 1);
    cloudIn.reset(new pcl::PointCloud<pcl::PointXYZI>());
    cloudOut.reset(new pcl::PointCloud<pcl::PointXYZI>());
    cloudTrim.reset(new pcl::PointCloud<pcl::PointXYZI>());
    transMatrix = pcl::getTransformation(0, 0, 0, M_PI, -M_PI_2, 0);
    // cout << transMatrix.matrix() << endl;
  }

  ~TransCloud() {}

  void cloudHandler(const sensor_msgs::PointCloud2ConstPtr &lidarCloudMsg) {
    header = lidarCloudMsg->header;
    pcl::fromROSMsg(*lidarCloudMsg, *cloudIn);
    if (cloudIn->is_dense == false) {
      vector<int> mapping;
      pcl::removeNaNFromPointCloud(*cloudIn, *cloudIn, mapping);
    }
    pcl::transformPointCloud(*cloudIn, *cloudOut, transMatrix);
    areaFilter();
    pcl::toROSMsg(*cloudTrim, transCloudMsg);
    transCloudMsg.header.frame_id = frame_id;
    transCloudMsg.header.stamp = header.stamp;
    pubLidarCloud.publish(transCloudMsg);
    resetParamsters();
  }

  void areaFilter() {
    if (cloudOut->empty())
      return;
    vector<int> indices;
    Eigen::Vector4f min_pt_vec(min_pt[0], min_pt[1], min_pt[2], min_pt[3]);
    Eigen::Vector4f max_pt_vec(max_pt[0], max_pt[1], max_pt[2], max_pt[3]);

    pcl::getPointsInBox(*cloudOut, min_pt_vec, max_pt_vec, indices);
    for (size_t i = 0; i < indices.size(); i++) {
      cloudTrim->points.push_back(cloudOut->points[indices[i]]);
    }
  }

  void resetParamsters() {
    cloudIn->clear();
    cloudOut->clear();
    cloudTrim->clear();
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "transCloud");
  TransCloud TC;
  ros::spin();
  return 0;
}