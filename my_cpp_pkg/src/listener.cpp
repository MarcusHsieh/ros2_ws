#include "my_cpp_pkg/listener.hpp"
#include <functional>

Listener::Listener() : Node("listener") {
  subscription_ = this->create_subscription<std_msgs::msg::String>(
    "chatter", 10,
    std::bind(&Listener::topic_callback, this, std::placeholders::_1)
  );
}

void Listener::topic_callback(const std_msgs::msg::String::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Listener>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
