#pragma once

#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/robot_hw.h"
#include "ros/publisher.h"

#include "lrr_base/lrr_connection.h"

#include "messages.pb.h"

namespace lrr_base {
class DriveBaseDriver : public hardware_interface::RobotHW,
                        public ConnectionParser {
public:
  DriveBaseDriver(ros::NodeHandle node_handle);
  void parse(OutgoingData &data);

  void write_joints();

  void initialize_state();

private:
  LRRConnection outgoing_connection_;
  LRRConnection incoming_connection_;

  // CONTROL INTERFACES
  struct Joint {
    double position;
    double velocity;
    double effort;
    double velocity_command;
    double position_offset;

    Joint()
        : position(0), velocity(0), effort(0), velocity_command(0),
          position_offset(std::numeric_limits<double>::quiet_NaN()) {}
  } joints_[4];

  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::VelocityJointInterface velocity_joint_interface_;
};
} // namespace lrr_base
