#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/u_int8.hpp"

#include <cstdint>

#include "heater.h"

#include "heater.c"

using std::placeholders::_1;

std::uint8_t temperature = 0;

class CopilotRVSubscriber : public rclcpp::Node {
 public:
  CopilotRVSubscriber() : Node("minimal_subscriber") {
    temperature_subscription_ = this->create_subscription<std_msgs::msg::UInt8>(
      "temperature", 10,
      std::bind(&CopilotRVSubscriber::temperature_callback, this, _1));
  }

  // Report monitor violations to the log.
  void func_on(float temp) {
    RCLCPP_INFO(this->get_logger(), "On: %f", temp);
  }

  // Report monitor violations to the log.
  void func_off(float temp) {
    RCLCPP_INFO(this->get_logger(), "Off: %f", temp);
  }

  // Needed so we can report messages to the log.
  static CopilotRVSubscriber& getInstance() {
    static CopilotRVSubscriber instance;
    return instance;
  }

 private:
  void temperature_callback(const std_msgs::msg::UInt8::SharedPtr msg) const {
    RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->data);
    temperature += msg->data;
    RCLCPP_INFO(this->get_logger(), "Executing Copilot monitors");
    step();
  }
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr temperature_subscription_;
};

// Pass monitor violations to the actual class, which has ways to communicate
// with other applications.
void heaton(float temp) {
  CopilotRVSubscriber::getInstance().func_on(temp);
}

void heatoff(float temp) {
  CopilotRVSubscriber::getInstance().func_off(temp);
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CopilotRVSubscriber>());
  rclcpp::shutdown();
  return 0;
}
