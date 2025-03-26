#include "my_cpp_pkg/talker.hpp"
#include <chrono>
#include <string>
#include <functional>

using namespace std::chrono_literals;

Talker::Talker() : Node("talker"), count_(0) {
  publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
  timer_ = this->create_wall_timer(1s, std::bind(&Talker::timer_callback, this));
}

void Talker::timer_callback() {
  auto message = std_msgs::msg::String();
  message.data = "Hello, world: " + std::to_string(count_++);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  publisher_->publish(message);
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Talker>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
