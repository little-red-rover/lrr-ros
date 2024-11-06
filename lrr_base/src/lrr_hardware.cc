#include <ros/ros.h>

#include "lrr_base/lrr_connection.h"
#include "ros/node_handle.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/LaserScan.h"

#include "lrr_base/lrr_hardware.h"

#include "messages.pb.h"

namespace lrr_base {

void lidar_callback(OutgoingData &data) { printf("Hit lidar_callback\n"); }

void imu_callback(OutgoingData &data) { printf("Hit imu_callback\n"); }

void joint_state_callback(OutgoingData &data) { printf("Hit imu_callback\n"); }

void joint_cmd_callback(OutgoingData &data) {
  // This connection is only used for sending commands, the callback should
  // never be hit
  std::fprintf(stderr,
               "Recieved unexpected message on command only connection.\n");
}

LRRHardware::LRRHardware(ros::NodeHandle node_handle)
    : node_handle_(node_handle), lidar_connection_(lidar_callback, LIDAR_DATA),
      imu_connection_(imu_callback, IMU_DATA),
      joint_state_connection_(joint_state_callback, JOINT_STATES_DATA),
      joint_cmd_connection_(joint_cmd_callback, NONE) {

  // Advertise ROS topics publishers
  imu_publisher_ = node_handle_.advertise<sensor_msgs::Imu>("imu/data_raw", 3);
  lidar_publisher_ = node_handle_.advertise<sensor_msgs::LaserScan>("scan", 3);
  joint_states_publisher_ =
      node_handle_.advertise<sensor_msgs::JointState>("joint_states", 3);
}

void LRRHardware::read_joints() {}

void LRRHardware::write_joints() {
  IncomingCommand cmd;

  // Right wheel
  cmd.mutable_joint_cmd()->set_joint(RIGHT_WHEEL);
  cmd.mutable_joint_cmd()->set_vel(0.0);
  cmd.mutable_joint_cmd()->mutable_time()->set_sec(ros::Time::now().sec);
  cmd.mutable_joint_cmd()->mutable_time()->set_nanosec(ros::Time::now().nsec);
  joint_cmd_connection_.send(cmd);

  // Left wheel
  cmd.mutable_joint_cmd()->set_joint(LEFT_WHEEL);
  cmd.mutable_joint_cmd()->set_vel(0.0);
  cmd.mutable_joint_cmd()->mutable_time()->set_sec(ros::Time::now().sec);
  cmd.mutable_joint_cmd()->mutable_time()->set_nanosec(ros::Time::now().nsec);
  joint_cmd_connection_.send(cmd);
}

}; // namespace lrr_base
