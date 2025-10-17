#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class SignalProcessor : public rclcpp::Node {
public:
    SignalProcessor() : Node("signal_processor") {
        // 创建订阅者，订阅正弦和方波信号
        sin_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
            "sin_signal", 10, std::bind(&SignalProcessor::sin_callback, this, std::placeholders::_1));
        square_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
            "square_signal", 10, std::bind(&SignalProcessor::square_callback, this, std::placeholders::_1));

        // 创建发布者，发布处理后的信号
        processed_publisher_ = this->create_publisher<std_msgs::msg::Float64>("processed_signal", 10);
    }

private:
    // 存储最新的正弦和方波值
    double latest_sin = 0.0;
    double latest_square = 0.0;

    // 正弦信号回调
    void sin_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        latest_sin = msg->data;
        process_signal();
    }

    // 方波信号回调
    void square_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        latest_square = msg->data;
        process_signal();
    }

    // 处理信号：同号输出正弦，否则输出0
    void process_signal() {
        double processed_val = 0.0;
        if ((latest_sin > 0 && latest_square > 0) || (latest_sin < 0 && latest_square < 0)) {
            processed_val = latest_sin;
        }
        auto msg = std_msgs::msg::Float64();
        msg.data = processed_val;
        processed_publisher_->publish(msg);
    }

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sin_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr square_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr processed_publisher_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SignalProcessor>());
    rclcpp::shutdown();
    return 0;
}