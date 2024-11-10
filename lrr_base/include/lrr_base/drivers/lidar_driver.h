#pragma once

#include "lrr_base/lrr_connection.h"

#include "laser_geometry/laser_geometry.h"
#include "ros/publisher.h"
#include <tf2_ros/transform_listener.h>

#include "messages.pb.h"

namespace lrr_base {
class LidarDriver : public ConnectionParser {
public:
  LidarDriver(ros::NodeHandle node_handle);
  void parse(OutgoingData &data);

private:
  ros::Publisher publisher_;
  LRRConnection connection_;

  sensor_msgs::LaserScan msg_;

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // LASER GEOMETRY
  laser_geometry::LaserProjection projector_;
};
} // namespace lrr_base
