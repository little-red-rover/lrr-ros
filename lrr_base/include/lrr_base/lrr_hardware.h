#pragma once

#include <ros/ros.h>

#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/robot_hw.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"

namespace lrr_base {
class LRRHardware : public hardware_interface::RobotHW {
public:
  LRRHardware(ros::NodeHandle node_handle);
  ~LRRHardware();

  void read_joints();
  void write_joints();

private:
  // NODE
  ros::NodeHandle node_handle_;

  // PUBLISHERS
  ros::Publisher imu_publisher_;
  ros::Publisher lidar_publisher_;

  // CONTROL INTERFACES
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::VelocityJointInterface velocity_joint_interface_;
};
}; // namespace lrr_base
