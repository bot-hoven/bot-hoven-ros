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

// Global promise to capture the time of the first feedback
std::promise<Clock::time_point> feedback_time_promise;
std::atomic<bool> feedback_received{false};

void feedback_callback(
  rclcpp_action::ClientGoalHandle<FollowJointTrajectory>::SharedPtr,
  const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback)
{
  (void)feedback; // Unused parameter, but required by the callback signature
  static bool first_feedback = true;
  if (first_feedback) {
    auto now = Clock::now();
    feedback_time_promise.set_value(now);
    feedback_received = true;
    first_feedback = false;
  }
}

TEST(RoboticControllerPerformanceMetricTest, GoalReceivalWithin20ms)
{
  rclcpp::init(0, nullptr);
  auto node = rclcpp::Node::make_shared("performance_test_node");

  auto action_client = rclcpp_action::create_client<FollowJointTrajectory>(
    node, "/right_hand_controller/follow_joint_trajectory");

  ASSERT_TRUE(action_client->wait_for_action_server(std::chrono::seconds(5)))
      << "Action server not available";

  FollowJointTrajectory::Goal goal;
  goal.goal_time_tolerance = rclcpp::Duration::from_seconds(0.1);
  goal.trajectory.joint_names = {"right_hand_stepper_joint"};
  
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.time_from_start = rclcpp::Duration::from_seconds(0.5);
  point.positions.push_back(1.0);
  goal.trajectory.points.push_back(point);

  rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions options;
  options.feedback_callback = std::bind(feedback_callback, std::placeholders::_1, std::placeholders::_2);

  auto send_time = Clock::now();
  auto goal_handle_future = action_client->async_send_goal(goal, options);
  rclcpp::spin_until_future_complete(node, goal_handle_future);

  auto goal_handle = goal_handle_future.get();
  ASSERT_NE(goal_handle, nullptr) << "Goal was rejected by the server";
  
  // Spin the node in a separate thread to process callbacks
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  // Capture the time after the goal is accepted
  auto accept_time = Clock::now();
  auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(accept_time - send_time).count();
  RCLCPP_INFO(node->get_logger(), "Time to receive goal: %ld ms", duration_ms);

  // Assert that the goal receival time is within the 20 ms threshold
  EXPECT_LE(duration_ms, 20) << "Goal receival exceeded 20 ms threshold";

  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}