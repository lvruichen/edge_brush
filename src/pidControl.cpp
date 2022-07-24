#include <Eigen/Core>
#include <edge_brush/edge.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
using namespace std;
#define sign(x) (((x) < 0) ? -1 : ((x) > 0))

enum edge_pos { LEFT = 1, RIGHT = -1 };
class PIDControl {
private:
  ros::NodeHandle nh_;
  ros::Subscriber subEdgeInfo;
  ros::Publisher pubCmdVel;
  int edgePos;
  float distance;
  float angleDiff;
  float targetDis;
  float spareDis;
  float c;
  float linearKpI;
  float angularKpI;
  float linearKpO;
  float angularKpO;
  float linearKpM;
  float angularKpM;

  float angularKi;
  float angularKd;
  float angularInt = 0;
  float lastErr = 0;
  float curErr = 0;
  string cmdTopic;
  Eigen::Vector2f dir_vec;
  Eigen::Vector2f targetPoint;

public:
  PIDControl() : distance(0), angleDiff(0) {
    nh_.param<std::string>("edge_brush/cmdTopic", cmdTopic, "/nav/cmd_vel");
    nh_.param<float>("edge_brush/targetDis", targetDis, 0.7);
    nh_.param<float>("edge_brush/spareDis", spareDis, 0.5);
    nh_.param<float>("edge_brush/linearKpI", linearKpI, 0.5);
    nh_.param<float>("edge_brush/angularKpI", angularKpI, 0.1);
    nh_.param<float>("edge_brush/angularKi", angularKi, 0.1);
    nh_.param<float>("edge_brush/angularKd", angularKd, 0.1);
    nh_.param<float>("edge_brush/linearKpO", linearKpO, 0.5);
    nh_.param<float>("edge_brush/angularKpO", angularKpO, 0.1);
    nh_.param<float>("edge_brush/linearKpM", linearKpM, 0.5);
    nh_.param<float>("edge_brush/angularKpM", angularKpM, 0.1);
    subEdgeInfo = nh_.subscribe<edge_brush::edge>(
        "/edge_info", 10, &PIDControl::edgeHandler, this);
    pubCmdVel = nh_.advertise<geometry_msgs::Twist>(cmdTopic, 1);
  }

  ~PIDControl() {}

  void edgeHandler(const edge_brush::edge::ConstPtr &edgeMsg) {
    this->distance = edgeMsg->distance;
    this->angleDiff = edgeMsg->angleDiff;
    this->edgePos = edgeMsg->edgePos;
    if (edgeMsg->Xdir < 0) {
      dir_vec(0) = -1 * edgeMsg->Xdir;
      dir_vec(1) = -1 * edgeMsg->Ydir;
    } else {
      dir_vec(0) = edgeMsg->Xdir;
      dir_vec(1) = edgeMsg->Ydir;
    }

    if (edgeMsg->edgePos > 0) {
      c = sqrt(dir_vec(0) * dir_vec(0) + dir_vec(1) * dir_vec(1)) *
          (distance - targetDis);
    } else {
      c = -1 * sqrt(dir_vec(0) * dir_vec(0) + dir_vec(1) * dir_vec(1)) *
          (distance - targetDis);
    }
    targetPoint(0) = 1;
    targetPoint(1) = (dir_vec(1) + c) / dir_vec(0);
    computeVel();
    // publishVel();
  }
  void publishVel() {
    geometry_msgs::Twist v;
    v.linear.x = min(targetPoint.norm() * linearKpM, float(0.5));
    v.linear.y = 0;
    v.linear.z = 0;
    v.angular.x = 0;
    v.angular.y = 0;
    v.angular.z = min(
        atan2(targetPoint(1), targetPoint(0)) * 180 / M_PI * angularKpM, 5.0);
    pubCmdVel.publish(v);
  }

  void computeVel() {
    if ((edgePos == LEFT && angleDiff < -20) ||
        (edgePos == RIGHT && angleDiff > 20)) {
      // has chance to collide
      angularInt = 0;
      lastErr = 0;
      curErr = 0;
      geometry_msgs::Twist v;
      v.linear.x = min(targetPoint.norm() * linearKpI, float(0.5));
      v.linear.y = 0;
      v.linear.z = 0;
      v.angular.x = 0;
      v.angular.y = 0;
      v.angular.z = min(
          atan2(targetPoint(1), targetPoint(0)) * 180 / M_PI * angularKpI, 5.0);
      pubCmdVel.publish(v);
    } else if ((edgePos == LEFT && angleDiff > 20) ||
               (edgePos == RIGHT && angleDiff < -20)) {
      // may unable to edge brush or collide the rear
      angularInt = 0;
      lastErr = 0;
      curErr = 0;
      if (edgePos > 0) {
        c = sqrt(dir_vec(0) * dir_vec(0) + dir_vec(1) * dir_vec(1)) *
            (distance - targetDis - spareDis);
      } else {
        c = -1 * sqrt(dir_vec(0) * dir_vec(0) + dir_vec(1) * dir_vec(1)) *
            (distance - targetDis - spareDis);
      }
      targetPoint(0) = 1;
      targetPoint(1) = (dir_vec(1) + c) / dir_vec(0);
      geometry_msgs::Twist v;
      v.linear.x = min(targetPoint.norm() * linearKpO, float(0.5));
      v.linear.y = 0;
      v.linear.z = 0;
      v.angular.x = 0;
      v.angular.y = 0;
      v.angular.z = min(
          atan2(targetPoint(1), targetPoint(0)) * 180 / M_PI * angularKpO, 5.0);
      pubCmdVel.publish(v);
    } else {
      // compute ki
      angularInt += targetPoint(1), targetPoint(0) * 180 / M_PI;
      // compute kd
      curErr = targetPoint(1), targetPoint(0) * 180 / M_PI;
      float errorDiff = curErr - lastErr;
      lastErr = curErr;
      geometry_msgs::Twist v;
      v.linear.x = min(targetPoint.norm() * linearKpM, float(0.5));
      v.linear.y = 0;
      v.linear.z = 0;
      v.angular.x = 0;
      v.angular.y = 0;
      float angular_vel =
          atan2(targetPoint(1), targetPoint(0)) * 180 / M_PI * angularKpM +
          angularInt * angularKi + angularKd * errorDiff;
      v.angular.z = min(angular_vel, float(5.0));

      pubCmdVel.publish(v);
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "PID_control");
  PIDControl pid;
  ROS_INFO("\033[1;32m----> PID Control Started.\033[0m");
  ros::spin();
  return 0;
}
