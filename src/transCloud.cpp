#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
using namespace std;

class TransCloud {
private:
  ros::NodeHandle nh_;
  ros::Subscriber subLidarCloud;
  ros::Publisher pubLidarCloud;
  string frame_id = "rslidar";
  string pointCloudTopic = "/rslidar_points";
  string pubTopic = "/rslidar_points_trans";
  std_msgs::Header header;
  sensor_msgs::PointCloud2 transCloudMsg;

public:
  TransCloud() {
    subLidarCloud = nh_.subscribe<sensor_msgs::PointCloud2>(
        pointCloudTopic, 10, &TransCloud::cloudHandler, this);
    pubLidarCloud = nh_.advertise<sensor_msgs::PointCloud2>(pubTopic, 1);
  }

  ~TransCloud() {}

  void cloudHandler(const sensor_msgs::PointCloud2ConstPtr &lidarCloudMsg) {
    transCloudMsg = *lidarCloudMsg;
    header = lidarCloudMsg->header;
    transCloudMsg.header.frame_id = frame_id;
    transCloudMsg.header.stamp = header.stamp;
    pubLidarCloud.publish(transCloudMsg);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "transCloud");
  TransCloud TC;
  ros::spin();
  return 0;
}