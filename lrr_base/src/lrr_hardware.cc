#include "hardware_interface/joint_state_interface.h"
#include "ros/node_handle.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/LaserScan.h"

#include <boost/assign/list_of.hpp>

#include "lrr_base/lrr_connection.h"
#include "lrr_base/lrr_hardware.h"

#include <tf2_ros/transform_listener.h>

#include "laser_geometry/laser_geometry.h"

#include "messages.pb.h"

namespace lrr_base {

#define deg_2_rad(angleInDegrees) ((angleInDegrees) * M_PI / 180.0)

static uint32_t i = 0;

LRRHardware::LRRHardware(ros::NodeHandle node_handle)
    : lidar_connection_(
          [this](OutgoingData &data) {
            assert(data.laser_size() != 0);
            for (LaserScan scan : data.laser()) {
              assert(scan.has_time());
              sensor_msgs::LaserScan ros_scan;

              // Convert protobuf message to ROS message
              ros_scan.header.frame_id = "lidar";
              ros_scan.header.stamp.sec = scan.time().sec();
              ros_scan.header.stamp.nsec = scan.time().nanosec();

              ros_scan.angle_min = deg_2_rad((float)scan.start_angle() / 100.0);
              ros_scan.angle_max = deg_2_rad((float)scan.end_angle() / 100.0);
              if (ros_scan.angle_max < ros_scan.angle_min) {
                ros_scan.angle_max = ros_scan.angle_max + 2 * M_PI;
              }

              ros_scan.angle_increment =
                  (ros_scan.angle_max - ros_scan.angle_min) /
                  scan.ranges_size();

              ros_scan.time_increment =
                  ros_scan.angle_increment / deg_2_rad((float)(scan.speed()));

              ros_scan.scan_time = ros_scan.time_increment * scan.ranges_size();

              ros_scan.range_min = 0.1;
              ros_scan.range_max = 8.0;

              ros_scan.ranges = std::vector<float>(scan.ranges().begin(),
                                                   scan.ranges().end());
              ros_scan.intensities = std::vector<float>(
                  scan.intensities().begin(), scan.intensities().end());

              for (auto &range : ros_scan.ranges) {
                range /= 1000.0;
              }

              lidar_publisher_.publish(ros_scan);
              // if (!tf_listener_.waitForTransform(
              //         ros_scan.header.frame_id, "/base_link",
              //         ros_scan.header.stamp +
              //             ros::Duration().fromSec(scan_in->ranges.size() *
              //                                     scan_in->time_increment),
              //         ros::Duration(1.0))) {
              //   return;
              // }
            }
          },
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
          NONE),
      node_handle_(node_handle), tf_listener_(tf_buffer_) {

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
