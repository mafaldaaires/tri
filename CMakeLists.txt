cmake_minimum_required(VERSION 3.5)
project(firstrobot_webots)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Besides the package specific dependencies we also need the `pluginlib` and `webots_ros2_driver`

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(webots_ros2_driver REQUIRED)

# Export the plugin configuration file
pluginlib_export_plugin_description_file(webots_ros2_driver my_robot_driver.xml)

# Obstacle avoider
include_directories(
  include
)
add_executable(obstacle_avoider
  src/ObstacleAvoider.cpp
)

ament_target_dependencies(obstacle_avoider
  rclcpp
  geometry_msgs
  sensor_msgs
)

install(TARGETS
  obstacle_avoider
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include
)

# Follower robot executable
add_executable(follower_robot
  src/follower_robot.cpp
)

ament_target_dependencies(follower_robot
  rclcpp
  sensor_msgs
  geometry_msgs
)

install(TARGETS
  follower_robot
  DESTINATION lib/${PROJECT_NAME}
)


# MyRobotDriver library
add_library(
  ${PROJECT_NAME}
  SHARED
  src/MyRobotDriver.cpp
  src/FollowerRobotDriver.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)
ament_target_dependencies(
  ${PROJECT_NAME}
  pluginlib
  rclcpp
  webots_ros2_driver
)
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# Install additional directories.
# Install additional directories
install(DIRECTORY
  include/
  launch
  resource
  worlds
  DESTINATION share/${PROJECT_NAME}/
)


install(
  FILES my_robot_driver.xml
  DESTINATION share/${PROJECT_NAME}
)
pluginlib_export_plugin_description_file(webots_ros2_driver my_robot_driver.xml)


ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_package()