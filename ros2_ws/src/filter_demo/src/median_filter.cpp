#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <vector>
#include <algorithm>

class MedianFilter : public rclcpp::Node {
public:
    MedianFilter() : Node("median_filter") {
        subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
            "raw_data", 10, std::bind(&MedianFilter::filter_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<std_msgs::msg::Float32>("median_filtered", 10);
        window_size_ = 5;
    }

private:
    void filter_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        window_.push_back(msg->data);
        if (window_.size() > window_size_) {
            window_.erase(window_.begin());
        }
        if (window_.size() == window_size_) {
            std::vector<float> sorted_window = window_;
            std::sort(sorted_window.begin(), sorted_window.end());
            float median = sorted_window[window_size_ / 2];
            auto filtered_msg = std_msgs::msg::Float32();
            filtered_msg.data = median;
            publisher_->publish(filtered_msg);
        }
    }

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    std::vector<float> window_;
    int window_size_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MedianFilter>());
    rclcpp::shutdown();
    return 0;
}