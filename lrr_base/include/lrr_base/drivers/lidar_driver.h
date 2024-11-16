#pragma once

#include "filters/filter_chain.hpp"
#include "geometry_msgs/Point.h"
#include "lrr_base/lrr_connection.h"

#include "ros/publisher.h"

#include "messages.pb.h"
#include "sensor_msgs/LaserScan.h"

namespace lrr_base {
class LidarDriver : public ConnectionParser {
public:
  LidarDriver(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle);
  void parse(OutgoingData &data);

private:
  ros::Publisher publisher_;
  ros::Publisher publisher_cloud_;
  LRRConnection connection_;

  sensor_msgs::LaserScan msg_;

  std::vector<geometry_msgs::Point> point_batch_;
  std::vector<float> intensity_batch_;

  filters::FilterChain<sensor_msgs::LaserScan> filter_chain_;
};
} // namespace lrr_base
