#include "firstrobot_webots/ObstacleAvoider.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "rclcpp/rclcpp.hpp"

#define MAX_RANGE 0.15
#define LINEAR_VELOCITY 0.075
#define ANGULAR_VELOCITY 0.5

ObstacleAvoider::ObstacleAvoider() : Node("obstacle_avoider"), 
                                     left_sensor_value(0.0), 
                                     right_sensor_value(0.0), 
                                     current_state(FORWARD){
                                      
  publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

  left_sensor_sub_ = create_subscription<sensor_msgs::msg::Range>(
      "/left_sensor", 1,
      [this](const sensor_msgs::msg::Range::SharedPtr msg){
        return this->leftSensorCallback(msg);
      }
  );

  right_sensor_sub_ = create_subscription<sensor_msgs::msg::Range>(
      "/right_sensor", 1,
      [this](const sensor_msgs::msg::Range::SharedPtr msg){
        return this->rightSensorCallback(msg);
      }
  );

  timer_ = create_wall_timer(
      std::chrono::milliseconds(1),
      std::bind(&ObstacleAvoider::controlLoop, this)
  );
}

void ObstacleAvoider::leftSensorCallback(const sensor_msgs::msg::Range::SharedPtr msg) {
  left_sensor_value = msg->range;
}

void ObstacleAvoider::rightSensorCallback(const sensor_msgs::msg::Range::SharedPtr msg) {
  right_sensor_value = msg->range;
}

void ObstacleAvoider::controlLoop() {
  auto command_message = std::make_unique<geometry_msgs::msg::Twist>();

  switch (current_state) {
    case FORWARD:
      command_message->linear.x = LINEAR_VELOCITY;
      command_message->angular.z = 0.0;
      if (right_sensor_value < 0.9 * MAX_RANGE) {
        current_state = ADJUST_DISTANCE;
      } else if (right_sensor_value > 0.9 * MAX_RANGE) {
        current_state = TURN_RIGHT;
      }
      break;

    case TURN_RIGHT:
      command_message->linear.x = 0.9 * LINEAR_VELOCITY;
      command_message->angular.z = -ANGULAR_VELOCITY;
      if (right_sensor_value < 0.9 * MAX_RANGE) {
        current_state = FORWARD;
      } 
      break;

    case ADJUST_DISTANCE:
      if (right_sensor_value < 0.9 * MAX_RANGE) {
        command_message->linear.x = 0.0;
        command_message->angular.z = ANGULAR_VELOCITY;
      }
      current_state = FORWARD;
      break;
  }

  publisher_->publish(std::move(command_message));
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto avoider = std::make_shared<ObstacleAvoider>();
  rclcpp::spin(avoider);
  rclcpp::shutdown();
  return 0;
}