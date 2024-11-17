# Little Red Rover ROS Driver

## Packages

This repo contains the following packages for working with Little Red Rover:

### lrr_base

Nodes:
* `lrr_base`
  * Implementation of a [ROS control](http://wiki.ros.org/ros_control) [hardware_interface](https://docs.ros.org/en/melodic/api/hardware_interface/html/c++/index.html).
    * Exposes a `JointPublisher` and a `JointVelocityController`
    * Makes the rover compatible with the standard ROS control [differential drive controller](http://wiki.ros.org/diff_drive_controller)
  * Implements a TCP client for each data source
    * Relays the recieved data into native ROS messages
    * Forwards joint state commands to the rover

Launch Files:
* `base.launch`
  * Starts the robot connection
  * Calls description.launch from lrr_description
  * Calls control.launch from lrr_control
  * Calls teleop.launch from lrr_control

### lrr_description

Contains a URDF (universal robot description format) description of the rover, generated from [the rover's CAD](https://cad.onshape.com/documents/3c9bcb798eb55dc89c3300c0/w/50990f614f9cfbc3dc016690/e/81981a5257c68cf876fffafa?renderMode=0&uiState=662964f9611b155af71d904d) using [onshape-to-robot](https://github.com/Rhoban/onshape-to-robot)

Launch Files:
* `description.launch`
  * Runs xacro on the robot description and loads it into a `robot_description` param for use in other launch files

### lrr_control

Launch Files:
* `control.launch`
  * Loads launch description using `description.launch` from lrr_description
  * Spawns hardware controllers using the controller_manager package
  * Runs an EKF for localization through the robot_localization package
  * Publishes robot state using the robot_state_publisher package

### lrr_navigation

Launch Files:
* `gmapping.launch`
* `amcl.launch`

### lrr_demos

WIP

### lrr_viz

Launch Files:
* `viz.launch`
  * Starts foxglove bridge
