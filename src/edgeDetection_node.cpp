#include "edge_brush/edgeDetection.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "edge_detection");
  EdgeDetection ED;
  ROS_INFO("\033[1;32m----> Edge Detection Started.\033[0m");
  ros::spin();
  return 0;
}