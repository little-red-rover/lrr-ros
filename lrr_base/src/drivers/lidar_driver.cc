#include "lrr_base/drivers/lidar_driver.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "messages.pb.h"
#include "ros/node_handle.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"
#include <cmath>

namespace lrr_base
{

#define deg_2_rad(angleInDegrees) ((angleInDegrees) * M_PI / 180.0)
#define OFFSET_X 5.9
#define OFFSET_Y -20.14
#define SUBSAMPLE_POINTS 720
#define SCAN_POINTS 360

  LidarDriver::LidarDriver(ros::NodeHandle node_handle)
      : connection_(this, LIDAR_DATA), tf_listener_(tf_buffer_)
  {
    publisher_ = node_handle.advertise<sensor_msgs::LaserScan>("scan", 3);

    msg_.header.frame_id = "lidar";
    msg_.range_min = 0.1;
    msg_.range_max = 8.0;
    msg_.scan_time = 0.1;
    msg_.time_increment = 0;
    msg_.angle_min = -M_PI;
    msg_.angle_max = M_PI;
    msg_.angle_increment = (msg_.angle_max - msg_.angle_min) / SCAN_POINTS;

    frame_ = "base_link";
  };

  void LidarDriver::parse(OutgoingData &data)
  {
    assert(data.laser_size() != 0);

    for (LaserScan scan : data.laser())
    {
      assert(scan.has_time());

      msg_.header.stamp.sec = scan.time().sec();
      msg_.header.stamp.nsec = scan.time().nanosec();

      double angle_min = deg_2_rad((float)scan.start_angle() / 100.0);
      double angle_max = deg_2_rad((float)scan.end_angle() / 100.0);
      if (angle_max < angle_min)
      {
        angle_max = angle_max + 2 * M_PI;
      }
      double angle_increment = (angle_max - angle_min) / scan.ranges_size();
      double time_increment = angle_increment / deg_2_rad((float)(scan.speed()));
      double scan_time = time_increment * scan.ranges_size();

      for (size_t i = 0; i < scan.ranges_size(); i++)
      {
        // Account for triangulation offsets, see LD20 data sheet
        float angle = angle_min + i * angle_increment;
        float distance = scan.ranges()[i] / 1000.0;
        float adj_x = scan.ranges()[i] + OFFSET_X;
        float adj_y = scan.ranges()[i] * 0.11923 + OFFSET_Y;
        double shift = atan(adj_y / adj_x) * 180.f / 3.14159;
        angle -= deg_2_rad(shift / 100.0);
        angle += M_PI;
        angle *= -1;

        geometry_msgs::Point in;
        in.x = cos(angle) * distance;
        in.y = sin(angle) * distance;

        point_batch_.push_back(in);
        intensity_batch_.push_back(scan.intensities()[i]);
      }
    }

    if (point_batch_.size() >= SUBSAMPLE_POINTS)
    {
      msg_.header.stamp = ros::Time().now();

      msg_.ranges.resize(SCAN_POINTS);
      msg_.intensities.resize(SCAN_POINTS);
      std::fill(msg_.ranges.begin(), msg_.ranges.end(), NAN);
      std::fill(msg_.intensities.begin(), msg_.intensities.end(), NAN);
      for (size_t i = 0; i < point_batch_.size(); i++)
      {
        geometry_msgs::Point point = point_batch_[i];
        size_t idx =
            (int)((atan2(point.y, point.x) / (2 * M_PI)) * SCAN_POINTS +
                  SCAN_POINTS / 2);
        float distance = sqrt(pow(point.x, 2) + pow(point.y, 2));

        msg_.ranges[idx] = distance;
        msg_.intensities[idx] = intensity_batch_[i];
      }

      publisher_.publish(msg_);

      intensity_batch_.clear();
      point_batch_.clear();
    }
  }
} // namespace lrr_base
