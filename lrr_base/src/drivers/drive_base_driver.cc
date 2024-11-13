#include "hardware_interface/joint_state_interface.h"

#include "messages.pb.h"
#include "ros/node_handle.h"
#include "sensor_msgs/JointState.h"
#include <boost/assign/list_of.hpp>

#include "lrr_base/drivers/drive_base_driver.h"

namespace lrr_base {

DriveBaseDriver::DriveBaseDriver(ros::NodeHandle node_handle)
    : outgoing_connection_(this, NONE),
      incoming_connection_(this, JOINT_STATES_DATA) {

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
}

void DriveBaseDriver::initialize_state() {
  joints_[0].position = 0.0;
  joints_[0].velocity = 0.0;
  joints_[0].effort = 0.0;

  joints_[1].position = 0.0;
  joints_[1].velocity = 0.0;
  joints_[1].effort = 0.0;
}

void DriveBaseDriver::parse(OutgoingData &data) {
  assert(data.has_joint_state());

  JointState js = data.joint_state();

  // Update the variables read by ros_control
  joints_[0].position = js.left_position();
  joints_[0].velocity = js.left_velocity();
  joints_[0].effort = js.left_effort();

  joints_[1].position = -js.right_position();
  joints_[1].velocity = -js.right_velocity();
  joints_[1].effort = -js.right_effort();
}

void DriveBaseDriver::write_joints() {
  IncomingCommand cmd;

  // Left wheel
  cmd.mutable_joint_cmd()->set_left_vel(joints_[0].velocity_command);
  cmd.mutable_joint_cmd()->mutable_time()->set_sec(ros::Time::now().sec);
  cmd.mutable_joint_cmd()->mutable_time()->set_nanosec(ros::Time::now().nsec);

  // Right wheel
  cmd.mutable_joint_cmd()->set_right_vel(joints_[1].velocity_command);
  cmd.mutable_joint_cmd()->mutable_time()->set_sec(ros::Time::now().sec);
  cmd.mutable_joint_cmd()->mutable_time()->set_nanosec(ros::Time::now().nsec);

  outgoing_connection_.send(cmd);
}
} // namespace lrr_base
