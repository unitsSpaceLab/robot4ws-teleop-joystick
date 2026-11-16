cmake_minimum_required(VERSION 3.16)
project(robot4ws_teleop_joystick)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot4ws_msgs REQUIRED)

include_directories(include)

add_executable(robot4ws_teleop_logi_controller_node src/teleop_joy_node.cpp)
ament_target_dependencies(robot4ws_teleop_logi_controller_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  robot4ws_msgs
)

install(TARGETS robot4ws_teleop_logi_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()