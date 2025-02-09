#include <gtest/gtest.h>
#include <chrono>
#include <future>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rcl_interfaces/msg/log.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using Clock = std::chrono::steady_clock;

class LogListener : public rclcpp::Node
{
public:
  LogListener() : Node("log_listener") {}

  void listen_for_logs()
  {
    subscription_ = this->create_subscription<rcl_interfaces::msg::Log>(
      "/rosout", 10,
      [this](const rcl_interfaces::msg::Log::SharedPtr msg) {
        std::string log_msg = msg->msg;
        std::string node_name = msg->name;
        process_log(node_name, log_msg);
      });

    executor_.add_node(this->get_node_base_interface());
    executor_.spin();
  }

  void process_log(const std::string &node_name, const std::string &log_msg)
  {
    if (log_msg.find("Received new action goal") != std::string::npos)
    {
      if (node_name.find("right_hand_controller") != std::string::npos)
        start_times_["right"] = Clock::now();
      else if (node_name.find("left_hand_controller") != std::string::npos)
        start_times_["left"] = Clock::now();
    }
    else if (log_msg.find("Accepted new action goal") != std::string::npos)
    {
      auto now = Clock::now();
      if (node_name.find("right_hand_controller") != std::string::npos && start_times_.count("right"))
      {
        right_hand_time_ = std::chrono::duration_cast<std::chrono::microseconds>(now - start_times_["right"]);
        RCLCPP_INFO(this->get_logger(), "Right Hand Controller - Time to accept goal: %ld us", right_hand_time_->count());
      }
      else if (node_name.find("left_hand_controller") != std::string::npos && start_times_.count("left"))
      {
        left_hand_time_ = std::chrono::duration_cast<std::chrono::microseconds>(now - start_times_["left"]);
        RCLCPP_INFO(this->get_logger(), "Left Hand Controller - Time to accept goal: %ld us", left_hand_time_->count());
      }

      if (right_hand_time_ && left_hand_time_)
      {
        average_time_ = std::chrono::microseconds((right_hand_time_->count() + left_hand_time_->count()) / 2);
        RCLCPP_INFO(this->get_logger(), "Average Goal Acceptance Time: %ld us", average_time_->count());
        executor_.cancel();
      }
    }
  }

  std::optional<std::chrono::microseconds> get_right_hand_time() const { return right_hand_time_; }
  std::optional<std::chrono::microseconds> get_left_hand_time() const { return left_hand_time_; }
  std::optional<std::chrono::microseconds> get_average_time() const { return average_time_; }

private:
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr subscription_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::unordered_map<std::string, Clock::time_point> start_times_;
  std::optional<std::chrono::microseconds> right_hand_time_;
  std::optional<std::chrono::microseconds> left_hand_time_;
  std::optional<std::chrono::microseconds> average_time_;
};

TEST(RoboticControllerPerformanceMetricTest, GoalAcceptanceWithin20ms)
{
  rclcpp::init(0, nullptr);
  auto timeout_us = std::chrono::microseconds(20000);
  auto log_listener = std::make_shared<LogListener>();

  // Start log listener in a separate thread
  std::thread log_thread([&]() { log_listener->listen_for_logs(); });

  std::this_thread::sleep_for(std::chrono::seconds(1));

  auto node = rclcpp::Node::make_shared("goal_sender");
  std::vector<std::string> controllers = {"/right_hand_controller/follow_joint_trajectory", 
                                          "/left_hand_controller/follow_joint_trajectory"};
  
  for (const auto &controller : controllers)
  {
    auto action_client = rclcpp_action::create_client<FollowJointTrajectory>(node, controller);
    ASSERT_TRUE(action_client->wait_for_action_server(std::chrono::seconds(5)))
        << "Action server not available for " << controller;
  
    FollowJointTrajectory::Goal goal;
    goal.goal_time_tolerance = rclcpp::Duration::from_seconds(0.1);
    
    if (controller.find("right") != std::string::npos)
    {
      goal.trajectory.joint_names = {"right_hand_stepper_joint", "right_hand_thumb_solenoid_joint", 
                                     "right_hand_thumb_servo_joint", "right_hand_index_finger_solenoid_joint", 
                                     "right_hand_index_finger_servo_joint", "right_hand_middle_finger_solenoid_joint", 
                                     "right_hand_middle_finger_servo_joint", "right_hand_ring_finger_solenoid_joint", 
                                     "right_hand_ring_finger_servo_joint", "right_hand_pinky_solenoid_joint", 
                                     "right_hand_pinky_servo_joint"};
    }
    else
    {
      goal.trajectory.joint_names = {"left_hand_stepper_joint", "left_hand_thumb_solenoid_joint", 
                                     "left_hand_thumb_servo_joint", "left_hand_index_finger_solenoid_joint", 
                                     "left_hand_index_finger_servo_joint", "left_hand_middle_finger_solenoid_joint", 
                                     "left_hand_middle_finger_servo_joint", "left_hand_ring_finger_solenoid_joint", 
                                     "left_hand_ring_finger_servo_joint", "left_hand_pinky_solenoid_joint", 
                                     "left_hand_pinky_servo_joint"};
    }
    
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.time_from_start = rclcpp::Duration::from_seconds(0.5);
    point.positions.resize(goal.trajectory.joint_names.size(), 1.0);
    goal.trajectory.points.push_back(point);
    
    auto goal_handle_future = action_client->async_send_goal(goal);
    ASSERT_EQ(rclcpp::spin_until_future_complete(node, goal_handle_future),
              rclcpp::FutureReturnCode::SUCCESS);
  }

  RCLCPP_INFO(node->get_logger(), "Waiting for goal acceptance logs...");
  auto start = Clock::now();
  while ((!log_listener->get_right_hand_time().has_value() || !log_listener->get_left_hand_time().has_value()) &&
         std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - start).count() < timeout_us.count())
  {
    std::this_thread::sleep_for(std::chrono::microseconds(1));
  }

  log_thread.join();
  rclcpp::shutdown();

  ASSERT_TRUE(log_listener->get_right_hand_time().has_value()) << "Failed to capture right hand goal acceptance log";
  ASSERT_TRUE(log_listener->get_left_hand_time().has_value()) << "Failed to capture left hand goal acceptance log";
  EXPECT_LE(log_listener->get_right_hand_time().value().count(), timeout_us.count()) << "Right hand goal acceptance exceeded 20 ms threshold";
  EXPECT_LE(log_listener->get_left_hand_time().value().count(), timeout_us.count()) << "Left hand goal acceptance exceeded 20 ms threshold";
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}