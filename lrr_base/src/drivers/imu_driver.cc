#include "messages.pb.h"
#include "ros/node_handle.h"
#include "sensor_msgs/Imu.h"

#include "lrr_base/drivers/imu_driver.h"

namespace lrr_base
{

  ImuDriver::ImuDriver(ros::NodeHandle node_handle)
      : connection_(this, IMU_DATA), gyro_bias_x_(0.0), gyro_bias_y_(0.0),
        gyro_bias_z_(0.0)
  {
    publisher_ = node_handle.advertise<sensor_msgs::Imu>("imu/data_raw", 3);

    ros::param::get("gyro_bias_x", gyro_bias_x_);
    ros::param::get("gyro_bias_y", gyro_bias_y_);
    ros::param::get("gyro_bias_z", gyro_bias_z_);

    msg_.header.frame_id = "base_link";

    // Disable orientation
    msg_.orientation_covariance.at(0) = -1;

    std::fill(std::begin(msg_.linear_acceleration_covariance),
              std::begin(msg_.linear_acceleration_covariance), 0);
    // TODO: Currently eyeballed from graphs, could use formal verification
    msg_.linear_acceleration_covariance[0] = 0.05;
    msg_.linear_acceleration_covariance[3] = 0.05;
    msg_.linear_acceleration_covariance[6] = 0.05;

    std::fill(std::begin(msg_.angular_velocity_covariance),
              std::begin(msg_.angular_velocity_covariance), 0);
    // TODO: Same for these
    msg_.angular_velocity_covariance[0] = 0.05;
    msg_.angular_velocity_covariance[3] = 0.05;
    msg_.angular_velocity_covariance[6] = 0.05;
  };

  void ImuDriver::parse(OutgoingData &data)
  {
    assert(data.has_imu() != 0);
    IMU imu = data.imu();
    assert(imu.has_time());

    msg_.header.stamp.sec = imu.time().sec();
    msg_.header.stamp.nsec = imu.time().nanosec();

    msg_.linear_acceleration.x = imu.accel_x();
    msg_.linear_acceleration.y = imu.accel_y();
    msg_.linear_acceleration.z = imu.accel_z();

    msg_.angular_velocity.x = imu.gyro_x() - gyro_bias_x_;
    msg_.angular_velocity.y = imu.gyro_y() - gyro_bias_y_;
    msg_.angular_velocity.z = imu.gyro_z() - gyro_bias_z_;

    publisher_.publish(msg_);
  }

} // namespace lrr_base
