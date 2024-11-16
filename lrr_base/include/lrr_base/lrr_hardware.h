#pragma once

#include <ros/ros.h>

#include "lrr_base/drivers/battery_driver.h"
#include "lrr_base/drivers/drive_base_driver.h"
#include "lrr_base/drivers/imu_driver.h"
#include "lrr_base/drivers/lidar_driver.h"
#include "ros/node_handle.h"

#include <tf2_ros/transform_listener.h>

namespace lrr_base {
class LRRHardware {
public:
  LRRHardware(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle);

  // DRIVERS
  LidarDriver lidar_driver;
  ImuDriver imu_driver;
  DriveBaseDriver drive_base_driver;
  BatteryDriver battery_driver;

private:
  // NODE
  ros::NodeHandle node_handle_;
  ros::NodeHandle private_nh;
};
}; // namespace lrr_base
