#ifndef FOLLOWER_ROBOT_DRIVER_HPP
#define FOLLOWER_ROBOT_DRIVER_HPP

#include "rclcpp/macros.hpp"
#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/WebotsNode.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace my_robot_driver {
class FollowerRobotDriver : public webots_ros2_driver::PluginInterface {
public:
  void step() override;
  void init(webots_ros2_driver::WebotsNode *node,
            std::unordered_map<std::string, std::string> &parameters) override;  // Use & to match base class

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
  geometry_msgs::msg::Twist cmd_vel_msg;
  webots_ros2_driver::WebotsNode *node_;

  WbDeviceTag right_motor;
  WbDeviceTag left_motor;
};
} // namespace my_robot_driver
#endif