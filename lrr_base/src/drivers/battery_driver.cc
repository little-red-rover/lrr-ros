#include "messages.pb.h"
#include "ros/node_handle.h"
#include "sensor_msgs/BatteryState.h"

#include "lrr_base/drivers/battery_driver.h"

namespace lrr_base {

BatteryDriver::BatteryDriver(ros::NodeHandle node_handle)
    : connection_(this, BATTERY_DATA), battery_readings_sum_(0.0) {
  publisher_ =
      node_handle.advertise<sensor_msgs::BatteryState>("battery_state", 3);
  msg_.header.frame_id = "/base_link";

  msg_.current = NAN;
  msg_.charge = NAN;
  msg_.capacity = NAN;
  msg_.design_capacity = NAN;

  msg_.power_supply_status =
      sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
  msg_.power_supply_health =
      sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
  msg_.power_supply_technology =
      sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;

  msg_.present = true;

  connection_.connect();
};

void BatteryDriver::parse(OutgoingData &data) {

  assert(data.has_battery());
  Battery battery = data.battery();
  assert(battery.has_time());

  // ADC readings can be noisy and are sensitive to motor noise
  // Apply a lowpass filter
  battery_readings_.push(battery.voltage());
  battery_readings_sum_ += battery.voltage();

  if (battery_readings_.size() >= 10) {
    battery_readings_sum_ -= battery_readings_.front();
    battery_readings_.pop();
  }
  float battery_voltage = battery_readings_sum_ / battery_readings_.size();

  msg_.header.stamp.sec = battery.time().sec();
  msg_.header.stamp.nsec = battery.time().nanosec();

  msg_.voltage = battery_voltage;
  msg_.percentage = (battery_voltage - 3.3) / (4.2 - 3.3);
  publisher_.publish(msg_);
}
} // namespace lrr_base
