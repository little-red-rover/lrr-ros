#pragma once

#include "ros/publisher.h"

#include "lrr_base/lrr_connection.h"

#include "messages.pb.h"
#include "sensor_msgs/BatteryState.h"
#include <queue>

namespace lrr_base {
class BatteryDriver : public ConnectionParser {
public:
  BatteryDriver(ros::NodeHandle node_handle);
  void parse(OutgoingData &data);

private:
  ros::Publisher publisher_;
  LRRConnection connection_;

  sensor_msgs::BatteryState msg_;

  std::queue<float> battery_readings_;
  float battery_readings_sum_;
};
} // namespace lrr_base
