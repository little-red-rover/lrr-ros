#include <ros/ros.h>

#include "hardware_interface/joint_state_interface.h"
#include "ros/node_handle.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/LaserScan.h"

#include <boost/assign/list_of.hpp>

#include "lrr_base/lrr_connection.h"
#include "lrr_base/lrr_hardware.h"

#include "messages.pb.h"

namespace lrr_base {

LRRHardware::LRRHardware(ros::NodeHandle node_handle)
    : node_handle_(node_handle),
      lidar_connection_(
          [this](OutgoingData &data) { printf("Hit lidar_callback\n"); },
          LIDAR_DATA),
      imu_connection_(
          [this](OutgoingData &data) { printf("Hit imu_callback\n"); },
          IMU_DATA),
      joint_state_connection_(
          [this](OutgoingData &data) {
            assert(data.has_joint_state());
            JointState js = data.joint_state();
            joints_[js.joint()].position = js.position();
            joints_[js.joint()].velocity = js.velocity();
            joints_[js.joint()].effort = js.effort();
          },
          JOINT_STATES_DATA),
      joint_cmd_connection_(
          [this](OutgoingData &data) {
            std::fprintf(
                stderr,
                "Recieved unexpected message on command only connection.\n");
          },
          NONE) {
  // Advertise ROS topics publishers
  imu_publisher_ = node_handle_.advertise<sensor_msgs::Imu>("imu/data_raw", 3);
  lidar_publisher_ = node_handle_.advertise<sensor_msgs::LaserScan>("scan", 3);
  joint_states_publisher_ =
      node_handle_.advertise<sensor_msgs::JointState>("joint_states", 3);

  // Register control interfaces
  ros::V_string joint_names =
      boost::assign::list_of("wheel_left")("wheel_right");

  for (unsigned int i = 0; i < joint_names.size(); i++) {
    hardware_interface::JointStateHandle joint_state_handle(
        joint_names[i], &joints_[i].position, &joints_[i].velocity,
        &joints_[i].effort);
    joint_state_interface_.registerHandle(joint_state_handle);

    hardware_interface::JointHandle joint_handle(joint_state_handle,
                                                 &joints_[i].velocity_command);
    velocity_joint_interface_.registerHandle(joint_handle);
  }
  registerInterface(&joint_state_interface_);
  registerInterface(&velocity_joint_interface_);

  ROS_INFO("LRR Base initialized");
}

void LRRHardware::write_joints() {
  IncomingCommand cmd;

  // Right wheel
  cmd.mutable_joint_cmd()->set_joint(RIGHT_WHEEL);
  cmd.mutable_joint_cmd()->set_vel(joints_[RIGHT_WHEEL].velocity_command);
  cmd.mutable_joint_cmd()->mutable_time()->set_sec(ros::Time::now().sec);
  cmd.mutable_joint_cmd()->mutable_time()->set_nanosec(ros::Time::now().nsec);
  joint_cmd_connection_.send(cmd);

  // Left wheel
  cmd.mutable_joint_cmd()->set_joint(LEFT_WHEEL);
  cmd.mutable_joint_cmd()->set_vel(joints_[LEFT_WHEEL].velocity_command);
  cmd.mutable_joint_cmd()->mutable_time()->set_sec(ros::Time::now().sec);
  cmd.mutable_joint_cmd()->mutable_time()->set_nanosec(ros::Time::now().nsec);
  joint_cmd_connection_.send(cmd);
}
}; // namespace lrr_base
