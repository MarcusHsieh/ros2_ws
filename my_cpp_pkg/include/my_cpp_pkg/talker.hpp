#ifndef MY_CPP_PKG_TALKER_HPP_
#define MY_CPP_PKG_TALKER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Talker : public rclcpp::Node {
public:
  Talker();
private:
  void timer_callback();
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
};

#endif  // MY_CPP_PKG_TALKER_HPP_
