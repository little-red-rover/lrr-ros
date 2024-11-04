#include <ros/ros.h>

#include "ros/node_handle.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"

#include "lrr_base/lrr_hardware.h"

#include "messages.pb.h"

namespace lrr_base {

LRRHardware::LRRHardware(ros::NodeHandle node_handle)
    : node_handle_(node_handle) {
  imu_publisher_ = node_handle_.advertise<sensor_msgs::Imu>("imu/data_raw", 3);
  lidar_publisher_ = node_handle_.advertise<sensor_msgs::LaserScan>("scan", 3);
}

LRRHardware::~LRRHardware() {}
}; // namespace lrr_base
