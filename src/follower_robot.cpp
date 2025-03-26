#include "firstrobot_webots/FollowerRobot.hpp"

#define MAX_RANGE 0.40
#define LINEAR_VELOCITY 0.075
#define ANGULAR_VELOCITY -0.5 

FollowerRobot::FollowerRobot() : Node("follower_robot"),
                                 left_sensor_value_(0.0),
                                 right_sensor_value_(0.0),
                                 robot_detector_sensor_value_(1.0),
                                 current_state_(FORWARD) {

  publisher_ = create_publisher<geometry_msgs::msg::Twist>("/follower_cmd_vel", 1);

  left_sensor_sub_ = create_subscription<sensor_msgs::msg::Range>(
      "/follower_ds0", 1,
      [this](const sensor_msgs::msg::Range::SharedPtr msg) {
        this->leftSensorCallback(msg);
      });

  right_sensor_sub_ = create_subscription<sensor_msgs::msg::Range>(
      "/follower_ds1", 1,
      [this](const sensor_msgs::msg::Range::SharedPtr msg) {
        this->rightSensorCallback(msg);
      });

  robot_detector_sensor_sub_ = create_subscription<sensor_msgs::msg::Range>(
    "/robot_detector", 1,
    [this](const sensor_msgs::msg::Range::SharedPtr msg) {
      this->robotDetectorSensorCallback(msg);
    });

  timer_ = create_wall_timer(
      std::chrono::milliseconds(1),
      std::bind(&FollowerRobot::controlLoop, this));
}

void FollowerRobot::leftSensorCallback(const sensor_msgs::msg::Range::SharedPtr msg) {
  left_sensor_value_ = msg->range;
}

void FollowerRobot::rightSensorCallback(const sensor_msgs::msg::Range::SharedPtr msg) {
  right_sensor_value_ = msg->range;
}

void FollowerRobot::robotDetectorSensorCallback(const sensor_msgs::msg::Range::SharedPtr msg) {
  robot_detector_sensor_value_ = msg->range;
}

void FollowerRobot::controlLoop() {
  auto command_message = std::make_unique<geometry_msgs::msg::Twist>();
  static double rotation_progress = 0.0;

  switch (current_state_) {

    case FORWARD:
      command_message->linear.x = LINEAR_VELOCITY;
      command_message->angular.z = 0.0;
      RCLCPP_INFO(this->get_logger(), "Robot detector value: %f", robot_detector_sensor_value_);

      if (robot_detector_sensor_value_ < 0.09) {
        RCLCPP_INFO(this->get_logger(), "Robot detected! Transitioning to FOLLOW_FIRST_ROBOT");
        current_state_ = ROTATE_180;
      } else if (left_sensor_value_ < 0.9 * MAX_RANGE) {
        current_state_ = ADJUST_DISTANCE;
      } else if (left_sensor_value_ > 0.9 * MAX_RANGE) {
        current_state_ = TURN_LEFT;
      }
      break;

    case TURN_LEFT:
      command_message->linear.x = 0.9 * LINEAR_VELOCITY;
      command_message->angular.z = -ANGULAR_VELOCITY;
      if (left_sensor_value_ < 0.9 * MAX_RANGE) {
        current_state_ = FORWARD;
      } 
      break;

    case ADJUST_DISTANCE:
      if (left_sensor_value_ < 0.9 * MAX_RANGE) {
        command_message->linear.x = 0.0;
        command_message->angular.z = ANGULAR_VELOCITY;
      }
      current_state_ = FORWARD;
      break;

    case ROTATE_180:
      command_message->linear.x = 0.02;
      command_message->angular.z = 0.5; 
      rotation_progress += 0.5 * 0.004;
      if (rotation_progress >= M_PI) { 
        current_state_ = FOLLOW_FIRST_ROBOT;
      }
      break;

    case FOLLOW_FIRST_ROBOT:
      command_message->linear.x = LINEAR_VELOCITY;
      command_message->angular.z = 0.0;
      if (right_sensor_value_ < 0.9 * 0.15) {
        current_state_ = ADJUST_DISTANCE_RIGHT;
      } else if (right_sensor_value_ > 0.9 * 0.15) {
        current_state_ = TURN_RIGHT;
      }
      break;

    case TURN_RIGHT:
      command_message->linear.x = 0.9 * LINEAR_VELOCITY;
      command_message->angular.z = ANGULAR_VELOCITY;
      if (right_sensor_value_ < 0.9 * 0.15) {
        current_state_ = FOLLOW_FIRST_ROBOT;
      }
      break;

    case ADJUST_DISTANCE_RIGHT:
      if (right_sensor_value_ < 0.9 * 0.15) {
        command_message->linear.x = 0.0;
        command_message->angular.z = -ANGULAR_VELOCITY;
      }
      current_state_ = FOLLOW_FIRST_ROBOT;
      break;
  }

  publisher_->publish(std::move(command_message));
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto follower = std::make_shared<FollowerRobot>();
  rclcpp::spin(follower);
  rclcpp::shutdown();
  return 0;
}