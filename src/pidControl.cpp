#include <Eigen/Core>
#include <edge_brush/edge.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
using namespace std;
#define sign(x) (((x) < 0) ? -1 : ((x) > 0))
class PIDControl {
private:
  ros::NodeHandle nh_;
  ros::Subscriber subEdgeInfo;
  ros::Publisher pubCmdVel;
  float distance;
  float angleDiff;
  float targetDis;
  float c;
  float linearKp;
  float angulerKp;
  string cmdTopic;
  Eigen::Vector2f dir_vec;
  Eigen::Vector2f targetPoint;

public:
  PIDControl() : distance(0), angleDiff(0) {
    nh_.param<std::string>("edge_brush/cmdTopic", cmdTopic, "/nav/cmd_vel");
    nh_.param<float>("edge_brush/targetDis", targetDis, 0.7);
    nh_.param<float>("edge_brush/linearKp", linearKp, 0.5);
    nh_.param<float>("edge_brush/angularKp", angulerKp, 0.1);
    subEdgeInfo = nh_.subscribe<edge_brush::edge>(
        "/edge_info", 10, &PIDControl::edgeHandler, this);
    pubCmdVel = nh_.advertise<geometry_msgs::Twist>(cmdTopic, 1);
  }

  ~PIDControl() {}

  void edgeHandler(const edge_brush::edge::ConstPtr &edgeMsg) {
    this->distance = edgeMsg->distance;
    this->angleDiff = edgeMsg->angleDiff;
    if (edgeMsg->Xdir > 0) {
      dir_vec(0) = edgeMsg->Xdir;
      dir_vec(1) = edgeMsg->Ydir;
      c = -1 * sqrt(dir_vec(0) * dir_vec(0) + dir_vec(1) * dir_vec(1)) *
          (distance - targetDis);
    } else {
      dir_vec(0) = -1 * edgeMsg->Xdir;
      dir_vec(1) = -1 * edgeMsg->Ydir;
      c = -1 * sqrt(dir_vec(0) * dir_vec(0) + dir_vec(1) * dir_vec(1)) *
          (distance - targetDis);
    }
    targetPoint(0) = 1;
    targetPoint(1) = (dir_vec(1) + c) / dir_vec(0);
    publishVel();
  }
  void publishVel() {
    geometry_msgs::Twist v;
    v.linear.x = targetPoint.norm() * linearKp;
    v.linear.y = 0;
    v.linear.z = 0;
    v.angular.x = 0;
    v.angular.y = 0;
    v.angular.z =
        atan2(targetPoint(1), targetPoint(0)) * 180 / M_PI * angulerKp;
    pubCmdVel.publish(v);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "PID_control");
  PIDControl pid;
  ROS_INFO("\033[1;32m----> PID Control Started.\033[0m");
  ros::spin();
  return 0;
}
