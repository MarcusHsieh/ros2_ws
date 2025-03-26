#ifndef MY_CPP_PKG_LISTENER_HPP_
#define MY_CPP_PKG_LISTENER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Listener : public rclcpp::Node {
public:
  Listener();
private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg);
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

#endif  // MY_CPP_PKG_LISTENER_HPP_
