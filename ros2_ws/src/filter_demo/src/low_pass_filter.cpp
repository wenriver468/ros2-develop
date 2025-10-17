#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

class LowPassFilter : public rclcpp::Node {
public:
    LowPassFilter() : Node("low_pass_filter") {
        subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
            "raw_data", 10, std::bind(&LowPassFilter::filter_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<std_msgs::msg::Float32>("low_pass_filtered", 10);
        alpha_ = 0.2;  // 滤波系数，可调整
        last_value_ = 0.0;
    }

private:
    void filter_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        float filtered = alpha_ * msg->data + (1 - alpha_) * last_value_;
        last_value_ = filtered;
        auto filtered_msg = std_msgs::msg::Float32();
        filtered_msg.data = filtered;
        publisher_->publish(filtered_msg);
    }

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    float alpha_;
    float last_value_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LowPassFilter>());
    rclcpp::shutdown();
    return 0;
}