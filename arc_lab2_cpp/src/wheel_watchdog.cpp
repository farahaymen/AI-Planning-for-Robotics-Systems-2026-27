// Lab 2, Exercise 2.5. You will not write C++ in this course, but you will read
// it, and reading it is much easier once you have compiled something.
//
// Two changes to make:
//   1. max_wheel_speed default 20.0 -> 16.5, the value the kinematics demanded.
//   2. Change the subscribed topic, rebuild, then change it back.
//
// Notice the structure is the same as Code 1.1: a class deriving from Node, a
// subscription with a callback, a declared parameter. Only the syntax and the
// build step differ.

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class WheelWatchdog : public rclcpp::Node
{
public:
  WheelWatchdog()
  : Node("wheel_watchdog")
  {
    this->declare_parameter<double>("max_wheel_speed", 20.0);
    this->declare_parameter<std::string>("joint_states_topic", "/joint_states");

    const auto topic = this->get_parameter("joint_states_topic").as_string();

    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      topic, 10,
      std::bind(&WheelWatchdog::on_joint_states, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "watching %s", topic.c_str());
  }

private:
  void on_joint_states(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    const double limit = this->get_parameter("max_wheel_speed").as_double();
    for (size_t i = 0; i < msg->name.size() && i < msg->velocity.size(); ++i) {
      if (std::abs(msg->velocity[i]) > limit) {
        RCLCPP_WARN(
          this->get_logger(), "%s at %.2f rad/s exceeds %.2f",
          msg->name[i].c_str(), msg->velocity[i], limit);
      }
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelWatchdog>());
  rclcpp::shutdown();
  return 0;
}
