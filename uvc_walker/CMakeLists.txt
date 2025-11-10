cmake_minimum_required(VERSION 3.8)
project(uvc_walker)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)

include_directories(
  include
)

add_executable(uvc_walker_node 
  src/uvc_walker.cpp
  src/main.cpp
)

ament_target_dependencies(uvc_walker_node
  rclcpp
  sensor_msgs
  dynamixel_sdk
)

install(TARGETS
  uvc_walker_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()




