#include <algorithm>
#include <cmath>
#include <deque>
#include <memory>
#include <numeric>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "trainros_interfaces/msg/stability.hpp"
#include "trainros_interfaces/msg/train_state.hpp"

class StabilityEvaluatorNode : public rclcpp::Node
{
public:
  StabilityEvaluatorNode() : Node("trainros_stability_evaluator")
  {
    warning_threshold_ = declare_parameter<double>("warning_threshold", 70.0);
    alarm_threshold_ = declare_parameter<double>("alarm_threshold", 40.0);
    window_size_ = declare_parameter<int>("window_size", 250);
    rms_alarm_acceleration_ = declare_parameter<double>("rms_alarm_acceleration", 3.0);

    publisher_ = create_publisher<trainros_interfaces::msg::Stability>("/stability", rclcpp::QoS(10).reliable());
    subscription_ = create_subscription<trainros_interfaces::msg::TrainState>(
      "/train_state",
      rclcpp::QoS(10).reliable(),
      std::bind(&StabilityEvaluatorNode::on_train_state, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Stability evaluator 已启动，窗口长度：%d", window_size_);
  }

private:
  void on_train_state(const trainros_interfaces::msg::TrainState::SharedPtr state)
  {
    const float acceleration_abs = std::abs(state->acceleration);
    window_.push_back(acceleration_abs);
    while (static_cast<int>(window_.size()) > window_size_) {
      window_.pop_front();
    }

    const double square_sum = std::accumulate(
      window_.begin(), window_.end(), 0.0,
      [](const double acc, const float value) { return acc + static_cast<double>(value) * value; });
    const float rms = window_.empty() ? 0.0F : static_cast<float>(std::sqrt(square_sum / window_.size()));
    const float peak = window_.empty() ? 0.0F : *std::max_element(window_.begin(), window_.end());
    const float score = static_cast<float>(
      std::max(0.0, 100.0 - static_cast<double>(rms) / rms_alarm_acceleration_ * 100.0));

    trainros_interfaces::msg::Stability msg;
    msg.header = state->header;
    msg.rms_acceleration = rms;
    msg.peak_acceleration = peak;
    msg.psd_value = 0.0F;
    msg.tsi = 0.0F;
    msg.score = score;
    msg.warning = msg.score < warning_threshold_;
    msg.alarm = msg.score < alarm_threshold_;
    msg.level = msg.alarm ? "alarm" : (msg.warning ? "warning" : "normal");
    publisher_->publish(msg);
  }

  double warning_threshold_ = 70.0;
  double alarm_threshold_ = 40.0;
  double rms_alarm_acceleration_ = 3.0;
  int window_size_ = 250;
  std::deque<float> window_;
  rclcpp::Publisher<trainros_interfaces::msg::Stability>::SharedPtr publisher_;
  rclcpp::Subscription<trainros_interfaces::msg::TrainState>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StabilityEvaluatorNode>());
  rclcpp::shutdown();
  return 0;
}
