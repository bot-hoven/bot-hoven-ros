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
        process_log(log_msg);
      });

    executor_.add_node(this->get_node_base_interface());
    executor_.spin();
  }

  void process_log(const std::string &log_msg)
  {
    static bool start_time_recorded = false;

    if (log_msg.find("Received new action goal") != std::string::npos && !start_time_recorded)
    {
      start_time_ = Clock::now();
      start_time_recorded = true;
    }
    else if (log_msg.find("Accepted new action goal") != std::string::npos && start_time_recorded)
    {
      auto accept_time = Clock::now();
      auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(accept_time - start_time_).count();
      RCLCPP_INFO(this->get_logger(), "Time to accept goal: %ld us", duration_us);

      recorded_time_ = duration_us;
      start_time_recorded = false;
      executor_.cancel(); // Stop spinning once the event is recorded
    }
  }

  std::optional<long> get_recorded_time() const
  {
    return recorded_time_;
  }

private:
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr subscription_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  Clock::time_point start_time_;
  std::optional<long> recorded_time_;
};

TEST(RoboticControllerPerformanceMetricTest, GoalAcceptanceWithin20ms)
{
  rclcpp::init(0, nullptr);
  auto log_listener = std::make_shared<LogListener>();

  // Start log listener in a separate thread
  std::thread log_thread([&]() { log_listener->listen_for_logs(); });

  // Give time for log listener to subscribe
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // Create node to send goal
  auto node = rclcpp::Node::make_shared("goal_sender");
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

  auto goal_handle_future = action_client->async_send_goal(goal);
  ASSERT_EQ(rclcpp::spin_until_future_complete(node, goal_handle_future),
            rclcpp::FutureReturnCode::SUCCESS);

  auto start = Clock::now();
  while (!log_listener->get_recorded_time().has_value() &&
         std::chrono::duration_cast<std::chrono::seconds>(Clock::now() - start).count() < 5)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  log_thread.join();
  rclcpp::shutdown();

  // Assert that the log timestamps were captured and within threshold
  ASSERT_TRUE(log_listener->get_recorded_time().has_value()) << "Failed to capture goal acceptance logs";
  EXPECT_LE(log_listener->get_recorded_time().value(), 20000) << "Goal acceptance exceeded 20 ms threshold";
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}