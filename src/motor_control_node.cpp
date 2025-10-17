#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class MotorControlNode : public rclcpp::Node {
public:
  MotorControlNode() : Node("motor_control_node") {
    // 初始化参数：目标转速、PID 系数、滤波系数
    target_speed_ = this->declare_parameter<double>("target_speed", 300.0);
    kp_ = this->declare_parameter<double>("kp", 0.01);
    ki_ = this->declare_parameter<double>("ki", 0.001);
    kd_ = this->declare_parameter<double>("kd", 0.0005);
    alpha_ = this->declare_parameter<double>("alpha", 0.2);

    // 订阅电机速度反馈
    speed_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
      "/simulated_motor_velocity", 10,
      std::bind(&MotorControlNode::speed_callback, this, std::placeholders::_1));

    // 发布控制扭矩
    torque_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
      "/motor_simulator_control_input", 10);

    // 定时器用于周期性更新控制逻辑（可选，这里主要靠回调触发）
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&MotorControlNode::control_loop, this));
  }

private:
  void speed_callback(const std_msgs::msg::Float64::SharedPtr msg) {
    // 低通滤波处理速度反馈
    filtered_speed_ = alpha_ * msg->data + (1 - alpha_) * filtered_speed_;
    last_speed_ = filtered_speed_;
  }

  void control_loop() {
    if (!rclcpp::ok()) return;

    // PID 计算：误差 = 目标转速 - 滤波后转速
    double error = target_speed_ - filtered_speed_;
    integral_ += error;
    double derivative = error - last_error_;
    last_error_ = error;

    // 计算控制扭矩
    double control_torque = kp_ * error + ki_ * integral_ + kd_ * derivative;

    // 发布控制扭矩
    auto torque_msg = std_msgs::msg::Float64();
    torque_msg.data = control_torque;
    torque_publisher_->publish(torque_msg);
  }

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr torque_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  double target_speed_;    // 目标转速
  double kp_, ki_, kd_;    // PID 系数
  double alpha_;           // 低通滤波系数
  double filtered_speed_ = 0.0;  // 滤波后转速
  double last_speed_ = 0.0;      // 上一时刻转速
  double last_error_ = 0.0;      // 上一时刻误差
  double integral_ = 0.0;        // 积分项
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorControlNode>());
  rclcpp::shutdown();
  return 0;
}