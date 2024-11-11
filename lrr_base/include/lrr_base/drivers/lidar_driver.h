#pragma once

#include "geometry_msgs/Point.h"
#include "lrr_base/lrr_connection.h"

#include "ros/publisher.h"
#include <tf2_ros/transform_listener.h>

#include "messages.pb.h"
#include "sensor_msgs/LaserScan.h"

#include <tf2_msgs/TFMessage.h>

namespace lrr_base {
class LidarDriver : public ConnectionParser {
public:
  LidarDriver(ros::NodeHandle node_handle);
  void parse(OutgoingData &data);

private:
  ros::Publisher publisher_;
  ros::Publisher publisher_cloud_;
  LRRConnection connection_;

  sensor_msgs::LaserScan msg_;

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::string frame_;

  std::vector<geometry_msgs::Point> point_batch_;
  std::vector<float> intensity_batch_;
};
} // namespace lrr_base
