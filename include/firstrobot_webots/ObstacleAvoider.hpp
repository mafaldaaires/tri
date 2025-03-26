#ifndef FIRSTROBOT_WEBOTS_OBSTACLEAVOIDER_HPP
#define FIRSTROBOT_WEBOTS_OBSTACLEAVOIDER_HPP

#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"

class ObstacleAvoider : public rclcpp::Node {
  public:
    enum State {
      FORWARD,
      TURN_RIGHT,
      ADJUST_DISTANCE
    };
  
    ObstacleAvoider();
  
  private:
    void leftSensorCallback(const sensor_msgs::msg::Range::SharedPtr msg);
    void rightSensorCallback(const sensor_msgs::msg::Range::SharedPtr msg);
    void controlLoop();
  
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr left_sensor_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr right_sensor_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
  
    double left_sensor_value;
    double right_sensor_value;
    State current_state;
  };
  
  #endif  // OBSTACLE_AVOIDER_HPP_