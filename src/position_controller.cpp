#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class ic2dPositionController : public rclcpp::Node {
   public:
    ic2dPositionController()
        : Node("ic2d_position_controller"), input_freq_(0.100), input_amp_(1.0) {
        
        using namespace std::chrono_literals;

        commands_.data.push_back(0);
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/position_controller/commands", 10);

        t_begin_ = this->now();
        timer_ = this->create_wall_timer(
            500ns, std::bind(&ic2dPositionController::timer_callback, this));
    }

   private:
    void timer_callback() {
        auto time_stamp = rclcpp::Duration(this->now() - t_begin_);
        commands_.data[0] = input_amp_ * sin(2 * M_PI * input_freq_ * time_stamp.seconds());
        publisher_->publish(commands_);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    std_msgs::msg::Float64MultiArray commands_;
    rclcpp::Time t_begin_;
    const double input_freq_;
    const double input_amp_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ic2dPositionController>());
    rclcpp::shutdown();
    return 0;
}