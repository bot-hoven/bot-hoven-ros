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
using Clock = rclcpp::Clock;
Clock ros_clock(RCL_ROS_TIME);

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
        start_times_["right"] = ros_clock.now();
      else if (node_name.find("left_hand_controller") != std::string::npos)
        start_times_["left"] = ros_clock.now();
    }
    else if (log_msg.find("Accepted new action goal") != std::string::npos)
    {
      auto now = ros_clock.now();
      if (node_name.find("right_hand_controller") != std::string::npos && start_times_.count("right"))
      {
        right_hand_time_ = rclcpp::Duration::from_nanoseconds((now - start_times_["right"]).nanoseconds());
        RCLCPP_INFO(this->get_logger(), "Right Hand Controller - Time to accept goal: %ld us", right_hand_time_->nanoseconds() / 1000);
      }
      else if (node_name.find("left_hand_controller") != std::string::npos && start_times_.count("left"))
      {
        left_hand_time_ = rclcpp::Duration::from_nanoseconds((now - start_times_["left"]).nanoseconds());
        RCLCPP_INFO(this->get_logger(), "Left Hand Controller - Time to accept goal: %ld us", left_hand_time_->nanoseconds() / 1000);
      }

      if (right_hand_time_ && left_hand_time_)
      {
        average_time_ = rclcpp::Duration::from_nanoseconds((*right_hand_time_ + *left_hand_time_).nanoseconds() / 2);
        RCLCPP_INFO(this->get_logger(), "Average Goal Acceptance Time: %ld us", average_time_->nanoseconds() / 1000);
        executor_.cancel();
      }
    }
  }

  std::optional<rclcpp::Duration> get_right_hand_time() const { return right_hand_time_; }
  std::optional<rclcpp::Duration> get_left_hand_time() const { return left_hand_time_; }
  std::optional<rclcpp::Duration> get_average_time() const { return average_time_; }

private:
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr subscription_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::unordered_map<std::string, rclcpp::Time> start_times_;
  std::optional<rclcpp::Duration> right_hand_time_;  // nanoseconds
  std::optional<rclcpp::Duration> left_hand_time_;   // nanoseconds
  std::optional<rclcpp::Duration> average_time_;     // nanoseconds
};

TEST(RoboticControllerPerformanceMetricTest, GoalAcceptanceWithin20ms)
{
  rclcpp::init(0, nullptr);
  auto timeout_ns = rclcpp::Duration::from_nanoseconds(20000 * 1000);
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
  auto start = ros_clock.now();
  while ((!log_listener->get_right_hand_time().has_value() || !log_listener->get_left_hand_time().has_value()) &&
         (ros_clock.now() - start).nanoseconds() < timeout_ns.nanoseconds())
  {
    std::this_thread::sleep_for(std::chrono::microseconds(1));
  }

  log_thread.join();
  rclcpp::shutdown();

  ASSERT_TRUE(log_listener->get_right_hand_time().has_value()) << "Failed to capture right hand goal acceptance log";
  ASSERT_TRUE(log_listener->get_left_hand_time().has_value()) << "Failed to capture left hand goal acceptance log";
  EXPECT_LE(log_listener->get_right_hand_time(), timeout_ns) << "Right hand goal acceptance exceeded 20 ms threshold";
  EXPECT_LE(log_listener->get_left_hand_time(), timeout_ns) << "Left hand goal acceptance exceeded 20 ms threshold";
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}