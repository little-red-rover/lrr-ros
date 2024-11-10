#include "lrr_base/drivers/lidar_driver.h"

#include "messages.pb.h"
#include "ros/node_handle.h"
#include "sensor_msgs/LaserScan.h"

namespace lrr_base {
#define deg_2_rad(angleInDegrees) ((angleInDegrees) * M_PI / 180.0)
#define OFFSET_X 5.9
#define OFFSET_Y -20.14

LidarDriver::LidarDriver(ros::NodeHandle node_handle)
    : connection_(this, LIDAR_DATA), tf_listener_(tf_buffer_) {
  publisher_ = node_handle.advertise<sensor_msgs::LaserScan>("scan", 3);

  msg_.header.frame_id = "lidar";
  msg_.range_min = 0.1;
  msg_.range_max = 8.0;
};

void LidarDriver::parse(OutgoingData &data) {
  assert(data.laser_size() != 0);
  for (LaserScan scan : data.laser()) {
    assert(scan.has_time());

    // Convert protobuf message to ROS message
    msg_.header.stamp.sec = scan.time().sec();
    msg_.header.stamp.nsec = scan.time().nanosec();

    msg_.angle_min = deg_2_rad((float)scan.start_angle() / 100.0);
    msg_.angle_max = deg_2_rad((float)scan.end_angle() / 100.0);
    if (msg_.angle_max < msg_.angle_min) {
      msg_.angle_max = msg_.angle_max + 2 * M_PI;
    }

    msg_.angle_increment =
        (msg_.angle_max - msg_.angle_min) / scan.ranges_size();

    msg_.time_increment =
        msg_.angle_increment / deg_2_rad((float)(scan.speed()));

    msg_.scan_time = msg_.time_increment * scan.ranges_size();

    msg_.ranges =
        std::vector<float>(scan.ranges().begin(), scan.ranges().end());
    msg_.intensities = std::vector<float>(scan.intensities().begin(),
                                          scan.intensities().end());

    for (auto &range : msg_.ranges) {
      // TODO: account for triangulation offsets
      // double offset_x = 5.9;
      // double offset_y = -20.14;
      range /= 1000.0;
    }

    publisher_.publish(msg_);

    // TODO: Account for skew from movement, combine readings into
    // single 360 scan for use with mapping packages if

    // Account for laser skew from a fast moving robot
    // LaserScan (/lidar frame) -> PointCloud2 (/map frame)
    sensor_msgs::PointCloud2 cloud;
    sensor_msgs::PointCloud2 local_cloud;
    std::string frame = "";
    if (tf_buffer_._frameExists("map")) {
      frame = "map";
    } else if (tf_buffer_._frameExists("odom")) {
      frame = "odom";
    } else {
      // TODO: Merge into batch message
      projector_.projectLaser(msg_, local_cloud);

      // cloud += local_cloud;
      continue;
    }

    try {
      tf_buffer_.lookupTransform(msg_.header.frame_id, frame,
                                 msg_.header.stamp +
                                     ros::Duration().fromSec(msg_.scan_time),
                                 ros::Duration(1.0));
      projector_.transformLaserScanToPointCloud("map", msg_, local_cloud,
                                                tf_buffer_);
    } catch (tf2::TransformException &ex) {
      ROS_WARN("Could NOT transform lidar frame : %s", ex.what());
    }
  }
  // Convert batch back into lidar frame
  // PointCloud2 (/map frame) -> LaserScan (/lidar frame)

  // Publish
}
} // namespace lrr_base
