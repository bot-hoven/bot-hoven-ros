#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <chrono>
#include <thread>
#include <vector>
#include <string>

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

// Number of "ON" commands (each "ON" is followed by an "OFF")
constexpr int num_on_commands = 280;
constexpr double command_period = 0.05;

class RapidCommandTest : public rclcpp::Node {
public:
  RapidCommandTest()
  : Node("single_finger_test"), on_count_(0), toggle_(false), test_completed_(false)
  {
    joint_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, "/left_hand_controller/follow_joint_trajectory");

    while (!joint_client_->wait_for_action_server(std::chrono::seconds(2))) {
      RCLCPP_WARN(this->get_logger(), "Waiting for action server...");
    }

    RCLCPP_INFO(this->get_logger(), "Action server available. Starting test...");
    start_time_ = this->now();
    send_command();
  }

private:
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr joint_client_;
  rclcpp::Time start_time_;
  size_t on_count_;
  bool toggle_;
  bool test_completed_;

  static constexpr double servo_center   = 0.0;

  // Based on URDF geometry of 0.2 -> -0.2 for finger position range.
  // The value of 0.15 was found empirically to coincide with a 15 degree angle for the servo used for testing
  static constexpr double servo_left = 0.15; 
  static constexpr double solenoid_off = 0.0;
  static constexpr double solenoid_on  = 1.0;

  void send_command() {
    if (on_count_ >= num_on_commands && !test_completed_) {
      RCLCPP_INFO(this->get_logger(), "Test completed. Sending final OFF command...");
      test_completed_ = true;
      send_final_off_command();
      return;
    }

    double target_servo = toggle_ ? servo_center : servo_left;
    double target_solenoid = toggle_ ? solenoid_on : solenoid_off;

    auto goal_msg = FollowJointTrajectory::Goal();
    goal_msg.trajectory.joint_names = {"left_hand_thumb_servo_joint", "left_hand_thumb_solenoid_joint"};

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {target_servo, target_solenoid};
    point.time_from_start = rclcpp::Duration::from_seconds(command_period);
    goal_msg.trajectory.points.push_back(point);

    auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    send_goal_options.result_callback = [this, target_servo, target_solenoid](const GoalHandle::WrappedResult &result) {
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(this->get_logger(), "Command %zu succeeded: Servo=%.2f, Solenoid=%.2f", on_count_, target_servo, target_solenoid);
      } else {
        RCLCPP_ERROR(this->get_logger(), "Command %zu failed", on_count_);
      }

      if (toggle_) {
        on_count_++;
      }
      toggle_ = !toggle_;
      std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(command_period * 1000)));
      send_command();
    };

    joint_client_->async_send_goal(goal_msg, send_goal_options);
  }

  void send_final_off_command() {
    auto goal_msg = FollowJointTrajectory::Goal();
    goal_msg.trajectory.joint_names = {"left_hand_thumb_servo_joint", "left_hand_thumb_solenoid_joint"};

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {servo_center, solenoid_off};
    point.time_from_start = rclcpp::Duration::from_seconds(command_period);
    goal_msg.trajectory.points.push_back(point);

    auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    send_goal_options.result_callback = [this](const GoalHandle::WrappedResult &result) {
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(this->get_logger(), "Final OFF command executed successfully.");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Final OFF command failed.");
      }
    };

    joint_client_->async_send_goal(goal_msg, send_goal_options);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RapidCommandTest>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
