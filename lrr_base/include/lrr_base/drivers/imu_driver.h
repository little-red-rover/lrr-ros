#pragma once

#include "ros/publisher.h"

#include "lrr_base/lrr_connection.h"

#include "messages.pb.h"
#include "sensor_msgs/Imu.h"

namespace lrr_base {
class ImuDriver : public ConnectionParser {
public:
  ImuDriver(ros::NodeHandle node_handle);
  void parse(OutgoingData &data);

private:
  ros::Publisher publisher_;
  LRRConnection connection_;

  sensor_msgs::Imu msg_;
};
} // namespace lrr_base
