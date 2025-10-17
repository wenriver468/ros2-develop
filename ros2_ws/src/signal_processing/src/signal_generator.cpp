#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath>

class SignalGenerator : public rclcpp::Node {
public:
    SignalGenerator() : Node("signal_generator") {
        // 创建发布者，分别发布正弦和方波信号
        sin_publisher_ = this->create_publisher<std_msgs::msg::Float64>("sin_signal", 10);
        square_publisher_ = this->create_publisher<std_msgs::msg::Float64>("square_signal", 10);

        // 定时器：控制发布频率（1ms一次，即1000Hz）
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&SignalGenerator::publish_signals, this));
    }

private:
    void publish_signals() {
        auto now = this->get_clock()->now();
        double time = now.seconds();

        // 10Hz正弦波：sin(2π*10*t)
        double sin_val = std::sin(2 * M_PI * 10 * time);
        auto sin_msg = std_msgs::msg::Float64();
        sin_msg.data = sin_val;
        sin_publisher_->publish(sin_msg);

        // 1Hz方波：周期1s，0.5s正、0.5s负
        double square_val = (std::fmod(time, 1.0) < 0.5) ? 1.0 : -1.0;
        auto square_msg = std_msgs::msg::Float64();
        square_msg.data = square_val;
        square_publisher_->publish(square_msg);
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr sin_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr square_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SignalGenerator>());
    rclcpp::shutdown();
    return 0;
}