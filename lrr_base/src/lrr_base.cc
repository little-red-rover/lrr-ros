#include "ros/ros.h"

#include "lrr_base/lrr_hardware.h"

using namespace lrr_base;

int main(int argc, char **argv) {
  ros::init(argc, argv, "lrr_base_node");

  ros::NodeHandle n;

  LRRHardware hardware(n);

  ros::spin();

  return 0;
}
