#ifndef FOLLOWER_ROBOT_HPP
#define FOLLOWER_ROBOT_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/range.hpp"

class FollowerRobot : public rclcpp::Node {
public:
  FollowerRobot();

private:
  enum State {
    FOLLOW_FIRST_ROBOT,
    FORWARD,
    TURN_LEFT,
    ADJUST_DISTANCE,
    FORWARD_RIGHT,
    TURN_RIGHT,
    ADJUST_DISTANCE_RIGHT,
    ROTATE_180
  };

  void leftSensorCallback(const sensor_msgs::msg::Range::SharedPtr msg);
  void rightSensorCallback(const sensor_msgs::msg::Range::SharedPtr msg);
  void robotDetectorSensorCallback(const sensor_msgs::msg::Range::SharedPtr msg);
  void controlLoop();

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr left_sensor_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr right_sensor_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr robot_detector_sensor_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double left_sensor_value_;
  double right_sensor_value_;
  double robot_detector_sensor_value_;
  State current_state_;
};

#endif  // FOLLOWER_ROBOT_HPP