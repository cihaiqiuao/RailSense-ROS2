#include <chrono>
#include <fstream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "trainros_interfaces/msg/system_status.hpp"
#include "trainros_interfaces/msg/train_state.hpp"

namespace
{
struct CpuTimes
{
  uint64_t idle = 0;
  uint64_t total = 0;
};

struct LatencySnapshot
{
  bool received = false;
  double latency_ms = -1.0;
  rclcpp::Time last_receive_time;
};

bool read_cpu_times(CpuTimes & out)
{
  std::ifstream file("/proc/stat");
  std::string cpu;
  uint64_t user = 0;
  uint64_t nice = 0;
  uint64_t system = 0;
  uint64_t idle = 0;
  uint64_t iowait = 0;
  uint64_t irq = 0;
  uint64_t softirq = 0;
  uint64_t steal = 0;
  if (!(file >> cpu >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> steal)) {
    return false;
  }
  out.idle = idle + iowait;
  out.total = user + nice + system + idle + iowait + irq + softirq + steal;
  return true;
}

bool read_memory_percent(float & memory_percent)
{
  std::ifstream file("/proc/meminfo");
  std::string key;
  uint64_t value = 0;
  std::string unit;
  uint64_t total = 0;
  uint64_t available = 0;
  while (file >> key >> value >> unit) {
    if (key == "MemTotal:") {
      total = value;
    } else if (key == "MemAvailable:") {
      available = value;
    }
    if (total > 0 && available > 0) {
      break;
    }
  }
  if (total == 0) {
    return false;
  }
  memory_percent = static_cast<float>((1.0 - static_cast<double>(available) / total) * 100.0);
  return true;
}

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

class MonitorNode : public rclcpp::Node
{
public:
  MonitorNode() : Node("trainros_monitor")
  {
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 1.0);
    topic_timeout_ms_ = declare_parameter<int>("topic_timeout_ms", 1000);

    publish_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    latency_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    status_pub_ = create_publisher<trainros_interfaces::msg::SystemStatus>(
      "/system_status", rclcpp::QoS(10).reliable());
    diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(10).reliable());

    rclcpp::SubscriptionOptions latency_options;
    latency_options.callback_group = latency_group_;
    const auto sensor_qos = rclcpp::SensorDataQoS();
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", sensor_qos,
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
        update_latency(imu_latency_, msg->header.stamp);
      },
      latency_options);
    gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gps/fix", rclcpp::QoS(10).reliable(),
      [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        update_latency(gps_latency_, msg->header.stamp);
      },
      latency_options);
    laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/laser/scan", sensor_qos,
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        update_latency(laser_latency_, msg->header.stamp);
      },
      latency_options);
    train_state_sub_ = create_subscription<trainros_interfaces::msg::TrainState>(
      "/train_state", rclcpp::QoS(10).reliable(),
      [this](const trainros_interfaces::msg::TrainState::SharedPtr msg) {
        update_latency(train_state_latency_, msg->header.stamp);
      },
      latency_options);

    read_cpu_times(last_cpu_);
    const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz_));
    timer_ = create_wall_timer(period, std::bind(&MonitorNode::publish_status, this), publish_group_);
    RCLCPP_INFO(get_logger(), "Monitor started, publish rate %.1f Hz", publish_rate_hz_);
  }

private:
  void update_latency(LatencySnapshot & snapshot, const builtin_interfaces::msg::Time & stamp)
  {
    const auto current = now();
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot.received = true;
    snapshot.last_receive_time = current;
    snapshot.latency_ms = (current - rclcpp::Time(stamp)).seconds() * 1000.0;
  }

  double receive_age_ms(const LatencySnapshot & snapshot, const rclcpp::Time & current) const
  {
    if (!snapshot.received) {
      return -1.0;
    }
    return (current - snapshot.last_receive_time).seconds() * 1000.0;
  }

  void add_latency_values(
    diagnostic_msgs::msg::DiagnosticStatus & diag,
    const std::string & name,
    const LatencySnapshot & snapshot,
    const rclcpp::Time & current)
  {
    add_key_value(diag.values, name + "_latency_ms", std::to_string(snapshot.latency_ms));
    add_key_value(diag.values, name + "_last_receive_age_ms", std::to_string(receive_age_ms(snapshot, current)));
  }

  bool topic_stale(const LatencySnapshot & snapshot, const rclcpp::Time & current) const
  {
    const double age = receive_age_ms(snapshot, current);
    return age < 0.0 || age > static_cast<double>(topic_timeout_ms_);
  }

  void publish_status()
  {
    trainros_interfaces::msg::SystemStatus status;
    status.header.stamp = now();

    CpuTimes current_cpu;
    if (read_cpu_times(current_cpu) && last_cpu_.total > 0 && current_cpu.total > last_cpu_.total) {
      const uint64_t total_delta = current_cpu.total - last_cpu_.total;
      const uint64_t idle_delta = current_cpu.idle - last_cpu_.idle;
      status.cpu_percent = static_cast<float>((1.0 - static_cast<double>(idle_delta) / total_delta) * 100.0);
      last_cpu_ = current_cpu;
    }

    read_memory_percent(status.memory_percent);
    status.status = "normal";
    status_pub_->publish(status);

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = status.header.stamp;
    diagnostic_msgs::msg::DiagnosticStatus diag;
    diag.name = "trainros_monitor";
    diag.hardware_id = "software";

    {
      std::lock_guard<std::mutex> lock(mutex_);
      const bool stale = topic_stale(imu_latency_, status.header.stamp) ||
        topic_stale(gps_latency_, status.header.stamp) ||
        topic_stale(laser_latency_, status.header.stamp) ||
        topic_stale(train_state_latency_, status.header.stamp);
      diag.level = stale ?
        diagnostic_msgs::msg::DiagnosticStatus::WARN :
        diagnostic_msgs::msg::DiagnosticStatus::OK;
      diag.message = stale ? "topic_latency_stale_or_missing" : "ok";

      add_latency_values(diag, "imu", imu_latency_, status.header.stamp);
      add_latency_values(diag, "gps", gps_latency_, status.header.stamp);
      add_latency_values(diag, "laser", laser_latency_, status.header.stamp);
      add_latency_values(diag, "train_state", train_state_latency_, status.header.stamp);
    }

    add_key_value(diag.values, "cpu_percent", std::to_string(status.cpu_percent));
    add_key_value(diag.values, "memory_percent", std::to_string(status.memory_percent));
    add_key_value(diag.values, "topic_timeout_ms", std::to_string(topic_timeout_ms_));
    array.status.push_back(diag);
    diag_pub_->publish(array);
  }

  double publish_rate_hz_ = 1.0;
  int topic_timeout_ms_ = 1000;
  CpuTimes last_cpu_;
  std::mutex mutex_;
  LatencySnapshot imu_latency_;
  LatencySnapshot gps_latency_;
  LatencySnapshot laser_latency_;
  LatencySnapshot train_state_latency_;
  rclcpp::CallbackGroup::SharedPtr publish_group_;
  rclcpp::CallbackGroup::SharedPtr latency_group_;
  rclcpp::Publisher<trainros_interfaces::msg::SystemStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<trainros_interfaces::msg::TrainState>::SharedPtr train_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MonitorNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
