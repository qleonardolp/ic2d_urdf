#include <chrono>
#include <functional>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

//using namespace std::chrono_literals;

// TODO: check on rqt_plot both topics: /effort_controller/commands/data[0] /joint_states/effort[0]

class ic2dEffortController : public rclcpp::Node
{
  public:
    ic2dEffortController()
    : Node("ic2d_effort_controller"), loop_rate_(1000), input_freq_(0.100)
    {
      // TODO: use std::bind and the create_wall_timer with a callback funct
      commands_.data.push_back(0);
      publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/effort_controller/commands", 10);

      t_begin_ = this->now();
      while (rclcpp::ok())
      {
        this->control_loop();
      }
    }

  private:
    void control_loop()
    {
        auto time_stamp = rclcpp::Duration(this->now() - t_begin_);
        commands_.data[0] = 10 * sin(
          2 * M_PI * input_freq_ * time_stamp.seconds());
        publisher_->publish(commands_);
        loop_rate_.sleep();
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    std_msgs::msg::Float64MultiArray commands_;
    rclcpp::Rate loop_rate_;
    rclcpp::Time t_begin_;
    const double input_freq_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ic2dEffortController>());
  rclcpp::shutdown();
  return 0;
}