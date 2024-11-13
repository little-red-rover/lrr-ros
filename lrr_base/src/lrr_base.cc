#include "controller_manager/controller_manager.h"
#include "ros/callback_queue.h"

#include "lrr_base/lrr_hardware.h"

#include <boost/chrono.hpp>

typedef boost::chrono::steady_clock time_source;

using namespace lrr_base;

void controlLoop(LRRHardware &hardware,
                 controller_manager::ControllerManager &controller_mgr,
                 time_source::time_point &last_time) {

  time_source::time_point this_time = time_source::now();
  boost::chrono::duration<double> elapsed_duration = this_time - last_time;
  ros::Duration elapsed(elapsed_duration.count());
  last_time = this_time;

  controller_mgr.update(ros::Time::now(), elapsed);
  hardware.drive_base_driver.write_joints();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "lrr_base_node");

  ros::NodeHandle node;
  LRRHardware hardware(node);

  controller_manager::ControllerManager controller_mgr(
      &hardware.drive_base_driver, node);

  ros::CallbackQueue lrr_queue;
  ros::AsyncSpinner lrr_spinner(1, &lrr_queue);
  time_source::time_point last_time = time_source::now();
  ros::TimerOptions control_timer(ros::Duration(1 / 50.0),
                                  boost::bind(controlLoop, boost::ref(hardware),
                                              boost::ref(controller_mgr),
                                              boost::ref(last_time)),
                                  &lrr_queue);
  ros::Timer control_loop = node.createTimer(control_timer);

  lrr_spinner.start();

  ros::spin();

  return 0;
}
