#include "lrr_base/drivers/battery_driver.h"
#include "lrr_base/drivers/drive_base_driver.h"
#include "lrr_base/drivers/imu_driver.h"
#include "lrr_base/drivers/lidar_driver.h"
#include "ros/node_handle.h"

#include "lrr_base/lrr_hardware.h"

#include <tf2_ros/transform_listener.h>

namespace lrr_base {

LRRHardware::LRRHardware(ros::NodeHandle node_handle,
                         ros::NodeHandle private_node_handle)
    : lidar_driver(node_handle, private_node_handle), imu_driver(node_handle),
      drive_base_driver(node_handle), battery_driver(node_handle),
      node_handle_(node_handle) {
  ROS_INFO("LRR Base initialized");
}
}; // namespace lrr_base
