#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <memory>
#include <mutex>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "trainros_gps_driver/gps_parser.hpp"

using namespace std::chrono_literals;
using trainros_gps_driver::RmcSample;
using trainros_gps_driver::parse_rmc;

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

int baud_to_flag(const int baud_rate)
{
  switch (baud_rate) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    default: return B115200;
  }
}

class SerialPort
{
public:
  ~SerialPort()
  {
    close_port();
  }

  bool open_port(const std::string & port, const int baud_rate)
  {
    close_port();
    fd_ = ::open(port.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      return false;
    }

    termios options {};
    if (tcgetattr(fd_, &options) != 0) {
      close_port();
      return false;
    }
    cfmakeraw(&options);
    const speed_t baud = baud_to_flag(baud_rate);
    cfsetispeed(&options, baud);
    cfsetospeed(&options, baud);
    options.c_cflag |= CLOCAL | CREAD;
    options.c_cflag &= ~CRTSCTS;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;
    if (tcsetattr(fd_, TCSANOW, &options) != 0) {
      close_port();
      return false;
    }
    return true;
  }

  void close_port()
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  bool is_open() const
  {
    return fd_ >= 0;
  }

  ssize_t read_some(uint8_t * data, const std::size_t size)
  {
    if (fd_ < 0) {
      return -1;
    }
    return ::read(fd_, data, size);
  }

private:
  int fd_ = -1;
};
}  // namespace

class GpsDriverNode : public rclcpp::Node
{
public:
  GpsDriverNode() : Node("trainros_gps_driver")
  {
    port_ = declare_parameter<std::string>("port", "/dev/ttyS9");
    frame_id_ = declare_parameter<std::string>("frame_id", "gps_link");
    baud_rate_ = declare_parameter<int>("baud_rate", 115200);
    reconnect_ms_ = declare_parameter<int>("reconnect_ms", 1000);
    no_fix_period_ms_ = declare_parameter<int>("no_fix_period_ms", 1000);
    diagnostics_period_ms_ = declare_parameter<int>("diagnostics_period_ms", 1000);

    publisher_ = create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", rclcpp::QoS(10).reliable());
    diagnostics_publisher_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", rclcpp::QoS(10).reliable());
    poll_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    no_fix_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    diagnostics_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_ = create_wall_timer(20ms, std::bind(&GpsDriverNode::poll_serial, this), poll_group_);
    no_fix_timer_ = create_wall_timer(
      std::chrono::milliseconds(no_fix_period_ms_),
      std::bind(&GpsDriverNode::publish_no_fix, this),
      no_fix_group_);
    diagnostics_timer_ = create_wall_timer(
      std::chrono::milliseconds(diagnostics_period_ms_),
      std::bind(&GpsDriverNode::publish_diagnostics, this),
      diagnostics_group_);

    RCLCPP_INFO(get_logger(), "GPS driver started, port: %s", port_.c_str());
  }

private:
  void ensure_open()
  {
    if (serial_.is_open()) {
      return;
    }
    const auto now_time = now();
    if (last_reconnect_.nanoseconds() != 0 &&
      (now_time - last_reconnect_).nanoseconds() < static_cast<int64_t>(reconnect_ms_) * 1000000LL)
    {
      return;
    }
    last_reconnect_ = now_time;
    ++reconnect_count_;
    if (!serial_.open_port(port_, baud_rate_)) {
      last_error_ = "open_failed";
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "GPS serial open failed: %s", port_.c_str());
      return;
    }
    last_error_.clear();
    RCLCPP_INFO(get_logger(), "GPS serial opened: %s", port_.c_str());
  }

  void poll_serial()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ensure_open();
    if (!serial_.is_open()) {
      return;
    }

    uint8_t tmp[256];
    const ssize_t n = serial_.read_some(tmp, sizeof(tmp));
    if (n > 0) {
      last_receive_time_ = now();
      buffer_.append(reinterpret_cast<const char *>(tmp), static_cast<std::size_t>(n));
      parse_lines();
      return;
    }
    if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      last_error_ = std::strerror(errno);
      RCLCPP_WARN(get_logger(), "GPS serial read failed, reconnecting: %s", std::strerror(errno));
      serial_.close_port();
    }
  }

  void parse_lines()
  {
    for (;;) {
      const auto pos = buffer_.find('\n');
      if (pos == std::string::npos) {
        return;
      }
      std::string line = buffer_.substr(0, pos);
      buffer_.erase(0, pos + 1);
      if (!line.empty() && line.back() == '\r') {
        line.pop_back();
      }

      RmcSample sample;
      if (!parse_rmc(line, sample)) {
        ++dropped_frames_;
        continue;
      }
      ++valid_frames_;

      sensor_msgs::msg::NavSatFix msg;
      msg.header.stamp = now();
      msg.header.frame_id = frame_id_;
      msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
      if (sample.valid_fix) {
        msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        msg.latitude = sample.latitude;
        msg.longitude = sample.longitude;
        msg.altitude = 0.0;
        has_valid_fix_ = true;
      } else {
        msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
      }
      publisher_->publish(msg);
    }
  }

  void publish_no_fix()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (has_valid_fix_) {
      return;
    }
    sensor_msgs::msg::NavSatFix msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
    msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    publisher_->publish(msg);
  }

  void publish_diagnostics()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "trainros_gps_driver";
    status.hardware_id = port_;
    if (!serial_.is_open()) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "serial_closed";
    } else if (valid_frames_ == 0) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "waiting_for_rmc";
    } else if (!has_valid_fix_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "no_fix";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "ok";
    }

    add_key_value(status.values, "serial_open", serial_.is_open() ? "true" : "false");
    add_key_value(status.values, "port", port_);
    add_key_value(status.values, "valid_frames", std::to_string(valid_frames_));
    add_key_value(status.values, "dropped_frames", std::to_string(dropped_frames_));
    add_key_value(status.values, "reconnect_count", std::to_string(reconnect_count_));
    add_key_value(
      status.values,
      "last_receive_time",
      last_receive_time_.nanoseconds() == 0 ? "never" : std::to_string(last_receive_time_.seconds()));
    add_key_value(status.values, "last_error", last_error_.empty() ? "none" : last_error_);
    add_key_value(status.values, "has_fix", has_valid_fix_ ? "true" : "false");

    array.status.push_back(status);
    diagnostics_publisher_->publish(array);
  }

  std::string port_;
  std::string frame_id_;
  std::string buffer_;
  int baud_rate_ = 115200;
  int reconnect_ms_ = 1000;
  int no_fix_period_ms_ = 1000;
  int diagnostics_period_ms_ = 1000;
  bool has_valid_fix_ = false;
  uint64_t valid_frames_ = 0;
  uint64_t dropped_frames_ = 0;
  uint64_t reconnect_count_ = 0;
  std::string last_error_ = "none";
  rclcpp::Time last_reconnect_;
  rclcpp::Time last_receive_time_;
  SerialPort serial_;
  std::mutex mutex_;
  rclcpp::CallbackGroup::SharedPtr poll_group_;
  rclcpp::CallbackGroup::SharedPtr no_fix_group_;
  rclcpp::CallbackGroup::SharedPtr diagnostics_group_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr no_fix_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GpsDriverNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
