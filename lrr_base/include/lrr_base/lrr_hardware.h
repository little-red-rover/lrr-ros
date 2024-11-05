#pragma once

#include <ros/ros.h>

#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/robot_hw.h"
#include "lrr_base/lrr_connection.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"

namespace lrr_base {
class LRRHardware : public hardware_interface::RobotHW {
public:
  LRRHardware(ros::NodeHandle node_handle);

  void read_joints();
  void write_joints();

private:
  // NODE
  ros::NodeHandle node_handle_;

  // PUBLISHERS
  ros::Publisher imu_publisher_;
  ros::Publisher lidar_publisher_;
  ros::Publisher joint_states_publisher_;

  // CONTROL INTERFACES
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::VelocityJointInterface velocity_joint_interface_;

  // COMMUNICATION INTERFACES
  // data
  LRRConnection lidar_connection_;
  LRRConnection imu_connection_;
  LRRConnection joint_state_connection_;
  // commands
  LRRConnection joint_cmd_connection_;
};
}; // namespace lrr_base
