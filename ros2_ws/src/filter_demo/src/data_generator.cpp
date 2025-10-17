#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <cmath>
#include <random>

class DataGenerator : public rclcpp::Node {
public:
    DataGenerator() : Node("data_generator") {
        publisher_ = this->create_publisher<std_msgs::msg::Float32>("raw_data", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),  // 100Hz频率
            std::bind(&DataGenerator::generate_data, this));
        time_ = 0.0;
        std::random_device rd;
        gen_ = std::mt19937(rd());
        dist_ = std::normal_distribution<double>(0, 0.2);  // 均值0、标准差0.2的噪声
    }

private:
    void generate_data() {
        // 生成1Hz正弦信号 + 随机噪声
        double signal = std::sin(2 * M_PI * 1 * time_);
        double noise = dist_(gen_);
        double raw_data = signal + noise;
        time_ += 0.01;  // 时间步长

        auto msg = std_msgs::msg::Float32();
        msg.data = raw_data;
        publisher_->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double time_;
    std::mt19937 gen_;
    std::normal_distribution<double> dist_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DataGenerator>());
    rclcpp::shutdown();
    return 0;
}