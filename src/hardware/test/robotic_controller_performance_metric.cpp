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

TEST(RoboticControllerPerformanceMetricTest, GoalAcceptanceWithin100ms)
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
  RCLCPP_INFO(node->get_logger(), "Time to receive goal acceptance: %ld ms", duration_ms);

  // Assert that the goal acceptance time is within the 100 ms threshold
  EXPECT_LE(duration_ms, 100) << "Goal acceptance exceeded 100 ms threshold";

  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
}

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
      auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(accept_time - start_time_).count();
      RCLCPP_INFO(this->get_logger(), "Time for goal acceptance: %ld ms", duration_ms);

      recorded_time_ = duration_ms;
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

TEST(RoboticControllerPerformanceMetricTest, GoalAcceptanceTimeFromLogs)
{
  rclcpp::init(0, nullptr);
  auto log_listener = std::make_shared<LogListener>();

  // Run log listener in a separate thread
  std::thread log_thread([&]() { log_listener->listen_for_logs(); });

  // Wait for log processing to complete (timeout after 5 seconds)
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
  EXPECT_LE(log_listener->get_recorded_time().value(), 2000) << "Controller took too long to accept the goal";
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}