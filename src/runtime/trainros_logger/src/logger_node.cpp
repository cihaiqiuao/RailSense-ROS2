#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "trainros_interfaces/msg/stability.hpp"

using namespace std::chrono_literals;

namespace
{
std::string json_escape(const std::string & input)
{
  std::string out;
  out.reserve(input.size());
  for (const char c : input) {
    switch (c) {
      case '\\': out += "\\\\"; break;
      case '"': out += "\\\""; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default: out += c; break;
    }
  }
  return out;
}

std::unordered_map<std::string, std::string> values_to_map(
  const std::vector<diagnostic_msgs::msg::KeyValue> & values)
{
  std::unordered_map<std::string, std::string> result;
  for (const auto & item : values) {
    result[item.key] = item.value;
  }
  return result;
}

uint64_t parse_u64(const std::unordered_map<std::string, std::string> & values, const std::string & key)
{
  const auto it = values.find(key);
  if (it == values.end()) {
    return 0;
  }
  try {
    return static_cast<uint64_t>(std::stoull(it->second));
  } catch (...) {
    return 0;
  }
}
}  // namespace

class LoggerNode : public rclcpp::Node
{
public:
  LoggerNode() : Node("trainros_logger")
  {
    log_dir_ = declare_parameter<std::string>("log_dir", "/userdata/trainros_logs");
    status_period_ms_ = declare_parameter<int>("status_period_ms", 1000);
    publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/log_status", rclcpp::QoS(10).reliable());

    event_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    status_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions event_options;
    event_options.callback_group = event_group_;
    diagnostics_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics",
      rclcpp::QoS(10).reliable(),
      std::bind(&LoggerNode::on_diagnostics, this, std::placeholders::_1),
      event_options);
    stability_sub_ = create_subscription<trainros_interfaces::msg::Stability>(
      "/stability",
      rclcpp::QoS(10).reliable(),
      std::bind(&LoggerNode::on_stability, this, std::placeholders::_1),
      event_options);

    open_log_file();
    timer_ = create_wall_timer(
      std::chrono::milliseconds(status_period_ms_),
      std::bind(&LoggerNode::publish_status, this),
      status_group_);

    write_event("logger", "info", "logger_started", "Logger 节点已启动", {{"log_path", log_path_}});
    RCLCPP_INFO(get_logger(), "Logger 已启动，日志目录：%s", log_dir_.c_str());
  }

private:
  struct DiagnosticSnapshot
  {
    int8_t level = -1;
    std::string message;
    bool has_serial_open = false;
    std::string serial_open;
    uint64_t reconnect_count = 0;
    uint64_t dropped_frames = 0;
    bool has_fix_known = false;
    std::string has_fix;
  };

  void open_log_file()
  {
    std::error_code ec;
    std::filesystem::create_directories(log_dir_, ec);
    if (ec) {
      log_ok_ = false;
      last_error_ = ec.message();
      return;
    }

    log_path_ = (std::filesystem::path(log_dir_) / "trainros_business.jsonl").string();
    log_stream_.open(log_path_, std::ios::app);
    log_ok_ = log_stream_.is_open();
    if (!log_ok_) {
      last_error_ = "无法打开日志文件";
    }
  }

  void write_event(
    const std::string & source,
    const std::string & level,
    const std::string & event,
    const std::string & message,
    const std::unordered_map<std::string, std::string> & fields = {})
  {
    if (!log_ok_) {
      return;
    }

    const auto stamp = now();
    log_stream_ << "{\"stamp_sec\":" << stamp.seconds()
                << ",\"source\":\"" << json_escape(source) << "\""
                << ",\"level\":\"" << json_escape(level) << "\""
                << ",\"event\":\"" << json_escape(event) << "\""
                << ",\"message\":\"" << json_escape(message) << "\"";
    for (const auto & [key, value] : fields) {
      log_stream_ << ",\"" << json_escape(key) << "\":\"" << json_escape(value) << "\"";
    }
    log_stream_ << "}\n";
    log_stream_.flush();
  }

  std::string level_to_string(const int8_t level) const
  {
    if (level == diagnostic_msgs::msg::DiagnosticStatus::OK) {
      return "info";
    }
    if (level == diagnostic_msgs::msg::DiagnosticStatus::WARN) {
      return "warning";
    }
    if (level == diagnostic_msgs::msg::DiagnosticStatus::ERROR) {
      return "error";
    }
    return "stale";
  }

  void on_diagnostics(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & status : msg->status) {
      const auto values = values_to_map(status.values);
      auto & snapshot = diagnostic_cache_[status.name];

      if (snapshot.level != status.level || snapshot.message != status.message) {
        write_event(
          status.name,
          level_to_string(status.level),
          "diagnostic_state_changed",
          status.message,
          {{"old_level", std::to_string(snapshot.level)}, {"new_level", std::to_string(status.level)}});
      }

      const auto serial_it = values.find("serial_open");
      if (serial_it != values.end() &&
        (!snapshot.has_serial_open || snapshot.serial_open != serial_it->second))
      {
        write_event(
          status.name,
          serial_it->second == "true" ? "info" : "error",
          serial_it->second == "true" ? "serial_opened" : "serial_closed",
          status.message,
          {{"serial_open", serial_it->second}, {"port", values.count("port") ? values.at("port") : ""}});
        snapshot.has_serial_open = true;
        snapshot.serial_open = serial_it->second;
      }

      const uint64_t reconnect_count = parse_u64(values, "reconnect_count");
      if (reconnect_count > snapshot.reconnect_count) {
        write_event(
          status.name,
          "warning",
          "serial_reconnect",
          status.message,
          {{"old_count", std::to_string(snapshot.reconnect_count)}, {"new_count", std::to_string(reconnect_count)}});
      }
      snapshot.reconnect_count = reconnect_count;

      const uint64_t dropped_frames = parse_u64(values, "dropped_frames");
      if (dropped_frames > snapshot.dropped_frames) {
        write_event(
          status.name,
          "warning",
          "parser_drop_frame",
          status.message,
          {{"old_count", std::to_string(snapshot.dropped_frames)}, {"new_count", std::to_string(dropped_frames)}});
      }
      snapshot.dropped_frames = dropped_frames;

      const auto fix_it = values.find("has_fix");
      if (fix_it != values.end() && (!snapshot.has_fix_known || snapshot.has_fix != fix_it->second)) {
        write_event(
          status.name,
          fix_it->second == "true" ? "info" : "warning",
          fix_it->second == "true" ? "gps_fix" : "gps_no_fix",
          status.message,
          {{"has_fix", fix_it->second}});
        snapshot.has_fix_known = true;
        snapshot.has_fix = fix_it->second;
      }

      snapshot.level = status.level;
      snapshot.message = status.message;
    }
  }

  void on_stability(const trainros_interfaces::msg::Stability::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (msg->level == last_stability_level_) {
      return;
    }
    last_stability_level_ = msg->level;

    const std::string event = msg->alarm ? "stability_alarm" :
      (msg->warning ? "stability_warning" : "stability_normal");
    const std::string level = msg->alarm ? "error" : (msg->warning ? "warning" : "info");
    write_event(
      "trainros_stability_evaluator",
      level,
      event,
      msg->level,
      {
        {"score", std::to_string(msg->score)},
        {"rms_acceleration", std::to_string(msg->rms_acceleration)},
        {"peak_acceleration", std::to_string(msg->peak_acceleration)},
      });
  }

  void publish_status()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "trainros_logger";
    status.level = log_ok_ ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = log_ok_ ? ("日志写入正常：" + log_path_) : ("日志写入失败：" + last_error_);
    array.status.push_back(status);

    publisher_->publish(array);
  }

  std::string log_dir_;
  std::string log_path_;
  std::string last_error_;
  std::string last_stability_level_;
  int status_period_ms_ = 1000;
  bool log_ok_ = false;
  std::ofstream log_stream_;
  std::unordered_map<std::string, DiagnosticSnapshot> diagnostic_cache_;
  std::mutex mutex_;
  rclcpp::CallbackGroup::SharedPtr event_group_;
  rclcpp::CallbackGroup::SharedPtr status_group_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr publisher_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_sub_;
  rclcpp::Subscription<trainros_interfaces::msg::Stability>::SharedPtr stability_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LoggerNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
