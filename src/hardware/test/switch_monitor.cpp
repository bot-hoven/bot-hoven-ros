#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <gpiod.hpp>              // C++ libgpiod header
#include <chrono>
#include <memory>
#include <string>

using namespace std::chrono_literals;

class SwitchMonitor : public rclcpp::Node {
public:
  SwitchMonitor()
  : Node("switch_monitor"), last_state_(1)  // Assume pull-up: default state is HIGH (1)
  {
    // Create a publisher on the "switch_state" topic.
    publisher_ = this->create_publisher<std_msgs::msg::String>("switch_state", 10);

    // Initialize the GPIO chip and request the line.
    // Adjust the chip name and line offset according to your Raspberry Pi.
    try {
      // "gpiochip0" is typically used on Raspberry Pi.
      chip_ = std::make_unique<gpiod::chip>("/dev/gpiochip4");
      // The line offset (17) corresponds to the desired GPIO pin.
      // Note: libgpiod uses the Linux kernel's line numbering,
      // which often corresponds to the Broadcom GPIO numbers.
      line_ = chip_->get_line(17);

      // gpiod::chip chip("gpiochip4");
      //           gpiod::line line_ = chip.get_line(17);
      //           int request_dir = gpiod::line_request::DIRECTION_INPUT;
      //           gpiod::line_request config = {"switch_monitor", request_dir, 0};
      //           line_.request(config, 0);

      // Request the line as an input. We assume that external pull-up resistors
      // (or hardware configuration) are in place since many Linux systems do not
      // allow software pull-ups via libgpiod.
      line_.request({"switch_monitor", gpiod::line_request::DIRECTION_INPUT, 0});
    } catch (const std::exception & e) {
      RCLCPP_FATAL(this->get_logger(), "Failed to initialize GPIO: %s", e.what());
      rclcpp::shutdown();
      return;
    }

    // Create a timer to periodically check the state of the switch (every 100 ms)
    timer_ = this->create_wall_timer(100ms, std::bind(&SwitchMonitor::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Switch Monitor node initialized. Monitoring GPIO line 17...");
  }

  ~SwitchMonitor() override {
    // libgpiod objects will automatically release the line and chip in their destructors.
  }

private:
  void timer_callback() {
    int current_state = line_.get_value();  // Returns 0 (LOW) if pressed, 1 (HIGH) if released
    if (current_state != last_state_) {
      std_msgs::msg::String msg;
      if (current_state == 0) {
        msg.data = "Pressed";
      } else {
        msg.data = "Released";
      }
      publisher_->publish(msg);
      RCLCPP_INFO(this->get_logger(), "Switch %s", msg.data.c_str());
      last_state_ = current_state;
    }
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  // libgpiod objects for GPIO access
  std::unique_ptr<gpiod::chip> chip_;
  gpiod::line line_;
  int last_state_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SwitchMonitor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
