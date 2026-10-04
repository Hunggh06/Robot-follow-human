// twist_to_rpm — cầu nối Leonardo-style /cmd_vel_mux (Twist) -> Kaia /motor/*_target_rpm.
//
// ESP32 Kaia KHÔNG nghe Twist, nó nghe 2 topic Float32 RPM trực tiếp.
// Node này áp đúng công thức differential-drive với số đo THẬT của bạn:
//   WHEEL_RADIUS = 0.0325, WHEEL_BASE = 0.2385, V_MAX = 0.36, W_MAX = 2*V_MAX/BASE.
#include <algorithm>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>

namespace {
constexpr float WHEEL_RADIUS = 0.0325f;
constexpr float WHEEL_BASE = 0.2385f;
constexpr float V_MAX = 0.36f;
constexpr float W_MAX = 2.0f * V_MAX / WHEEL_BASE;  // ~3.02 rad/s
constexpr float RPM_FACTOR = 60.0f / (2.0f * static_cast<float>(M_PI));
}  // namespace

class TwistToRpm : public rclcpp::Node {
public:
  TwistToRpm() : Node("twist_to_rpm") {
    pub_l_ = create_publisher<std_msgs::msg::Float32>("/motor/left_target_rpm", 10);
    pub_r_ = create_publisher<std_msgs::msg::Float32>("/motor/right_target_rpm", 10);
    sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_mux", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr m) { on_cmd(m); });
    RCLCPP_INFO(get_logger(), "twist_to_rpm ready (base=%.4f r=%.4f vmax=%.2f)",
      WHEEL_BASE, WHEEL_RADIUS, V_MAX);
  }

private:
  void on_cmd(const geometry_msgs::msg::Twist::SharedPtr m) {
    float v = std::clamp(static_cast<float>(m->linear.x), -V_MAX, V_MAX);
    float w = std::clamp(static_cast<float>(m->angular.z), -W_MAX, W_MAX);
    if (std::fabs(v) < 1e-2f) w *= 0.8f;  // luật Leonardo: bớt xoay tại chỗ
    float v_r = v + (WHEEL_BASE / 2.0f) * w;
    float v_l = v - (WHEEL_BASE / 2.0f) * w;
    float peak = std::max(std::fabs(v_r), std::fabs(v_l));
    if (peak > V_MAX) { v_r *= V_MAX / peak; v_l *= V_MAX / peak; }
    std_msgs::msg::Float32 ml, mr;
    ml.data = (v_l / WHEEL_RADIUS) * RPM_FACTOR;
    mr.data = (v_r / WHEEL_RADIUS) * RPM_FACTOR;
    pub_l_->publish(ml);
    pub_r_->publish(mr);
  }
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_l_, pub_r_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TwistToRpm>());
  rclcpp::shutdown();
  return 0;
}
