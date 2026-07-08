#include <algorithm>
#include <cmath>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "trainros_stability_evaluator/stability_metrics.hpp"
#include "trainros_interfaces/msg/stability.hpp"
#include "trainros_interfaces/msg/train_state.hpp"

namespace
{
void add_key_value(
  std::vector<diagnostic_msgs::msg::KeyValue> & values,
  const std::string & key,
  const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  values.push_back(item);
}
}  // namespace

class StabilityEvaluatorNode : public rclcpp::Node
{
public:
  StabilityEvaluatorNode() : Node("trainros_stability_evaluator")
  {
    config_.warning_threshold = declare_parameter<double>("warning_threshold", 70.0);
    config_.alarm_threshold = declare_parameter<double>("alarm_threshold", 40.0);
    config_.window_size = declare_parameter<int>("window_size", 250);
    declare_parameter<double>("rms_alarm_acceleration", 3.0);
    config_.sample_rate_hz = declare_parameter<double>("sample_rate_hz", 50.0);
    config_.min_window_samples = declare_parameter<int>("min_window_samples", 50);
    config_.psd_min_hz = declare_parameter<double>("psd_min_hz", 0.5);
    config_.psd_max_hz = declare_parameter<double>("psd_max_hz", 20.0);
    config_.rms_weight = declare_parameter<double>("rms_weight", 1.0);
    config_.peak_weight = declare_parameter<double>("peak_weight", 0.3);
    config_.psd_weight = declare_parameter<double>("psd_weight", 0.2);
    config_.tsi_alarm_value = declare_parameter<double>("tsi_alarm_value", 5.0);
    diagnostics_period_ms_ = declare_parameter<int>("diagnostics_period_ms", 1000);
    evaluator_ = std::make_unique<trainros_stability_evaluator::StabilityWindowEvaluator>(config_);

    publisher_ = create_publisher<trainros_interfaces::msg::Stability>("/stability", rclcpp::QoS(10).reliable());
    diagnostics_publisher_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", rclcpp::QoS(10).reliable());
    subscription_ = create_subscription<trainros_interfaces::msg::TrainState>(
      "/train_state",
      rclcpp::QoS(10).reliable(),
      std::bind(&StabilityEvaluatorNode::on_train_state, this, std::placeholders::_1));
    parameter_callback_ = add_on_set_parameters_callback(
      std::bind(&StabilityEvaluatorNode::on_parameters, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Stability evaluator 已启动，窗口长度：%d", config_.window_size);
  }

private:
  void on_train_state(const trainros_interfaces::msg::TrainState::SharedPtr state)
  {
    evaluator_->add_sample(state->acceleration);
    last_metrics_ = evaluator_->evaluate();

    trainros_interfaces::msg::Stability msg;
    msg.header = state->header;
    msg.rms_acceleration = last_metrics_.rms_acceleration;
    msg.peak_acceleration = last_metrics_.peak_acceleration;
    msg.psd_value = last_metrics_.psd_value;
    msg.tsi = last_metrics_.tsi;
    msg.score = last_metrics_.score;
    msg.warning = last_metrics_.warning;
    msg.alarm = last_metrics_.alarm;
    msg.level = last_metrics_.level;
    publisher_->publish(msg);

    const auto now = std::chrono::steady_clock::now();
    if (last_diagnostics_wall_time_.time_since_epoch().count() == 0 ||
      now - last_diagnostics_wall_time_ >= std::chrono::milliseconds(diagnostics_period_ms_))
    {
      last_diagnostics_wall_time_ = now;
      publish_diagnostics(state->header.stamp);
    }
  }

  rcl_interfaces::msg::SetParametersResult on_parameters(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    auto next_config = config_;
    for (const auto & parameter : parameters) {
      const auto & name = parameter.get_name();
      if (name == "warning_threshold") {
        next_config.warning_threshold = parameter.as_double();
      } else if (name == "alarm_threshold") {
        next_config.alarm_threshold = parameter.as_double();
      } else if (name == "window_size") {
        next_config.window_size = parameter.as_int();
      } else if (name == "sample_rate_hz") {
        next_config.sample_rate_hz = parameter.as_double();
      } else if (name == "min_window_samples") {
        next_config.min_window_samples = parameter.as_int();
      } else if (name == "psd_min_hz") {
        next_config.psd_min_hz = parameter.as_double();
      } else if (name == "psd_max_hz") {
        next_config.psd_max_hz = parameter.as_double();
      } else if (name == "rms_weight") {
        next_config.rms_weight = parameter.as_double();
      } else if (name == "peak_weight") {
        next_config.peak_weight = parameter.as_double();
      } else if (name == "psd_weight") {
        next_config.psd_weight = parameter.as_double();
      } else if (name == "tsi_alarm_value") {
        next_config.tsi_alarm_value = parameter.as_double();
      } else if (name == "diagnostics_period_ms") {
        diagnostics_period_ms_ = std::max(10, static_cast<int>(parameter.as_int()));
      }
    }

    config_ = next_config;
    evaluator_->set_config(config_);
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  void publish_diagnostics(const builtin_interfaces::msg::Time & stamp)
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = stamp;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "trainros_stability_evaluator";
    status.hardware_id = "software";
    status.level = last_metrics_.alarm ? diagnostic_msgs::msg::DiagnosticStatus::ERROR :
      (last_metrics_.warning ? diagnostic_msgs::msg::DiagnosticStatus::WARN :
      diagnostic_msgs::msg::DiagnosticStatus::OK);
    status.message = last_metrics_.level;
    add_key_value(status.values, "window_samples", std::to_string(last_metrics_.sample_count));
    add_key_value(status.values, "rms_acceleration", std::to_string(last_metrics_.rms_acceleration));
    add_key_value(status.values, "peak_acceleration", std::to_string(last_metrics_.peak_acceleration));
    add_key_value(status.values, "psd_value", std::to_string(last_metrics_.psd_value));
    add_key_value(status.values, "tsi", std::to_string(last_metrics_.tsi));
    add_key_value(status.values, "score", std::to_string(last_metrics_.score));
    add_key_value(status.values, "level", last_metrics_.level);
    array.status.push_back(status);
    diagnostics_publisher_->publish(array);
  }

  trainros_stability_evaluator::StabilityConfig config_;
  trainros_stability_evaluator::StabilityMetrics last_metrics_;
  std::unique_ptr<trainros_stability_evaluator::StabilityWindowEvaluator> evaluator_;
  int diagnostics_period_ms_ = 1000;
  std::chrono::steady_clock::time_point last_diagnostics_wall_time_;
  rclcpp::Publisher<trainros_interfaces::msg::Stability>::SharedPtr publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Subscription<trainros_interfaces::msg::TrainState>::SharedPtr subscription_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StabilityEvaluatorNode>());
  rclcpp::shutdown();
  return 0;
}
