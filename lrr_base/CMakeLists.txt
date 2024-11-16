cmake_minimum_required(VERSION 3.0.2)
project(lrr_base)

find_package(Boost REQUIRED COMPONENTS chrono)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  controller_manager
  hardware_interface
  tf2
  tf2_geometry_msgs
)

catkin_package()

install(DIRECTORY config launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIRS}
)

add_executable(
    lrr_base 
    src/lrr_base.cc 
    src/lrr_hardware.cc 
    src/lrr_connection.cc 
    src/drivers/lidar_driver.cc
    src/drivers/imu_driver.cc
    src/drivers/battery_driver.cc
    src/drivers/drive_base_driver.cc
)

add_dependencies(lrr_base ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} proto )
target_link_libraries(lrr_base
  ${catkin_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  proto
)

add_subdirectory(proto)
