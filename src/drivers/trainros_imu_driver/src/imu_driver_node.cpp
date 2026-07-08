#include <algorithm>
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
#include "sensor_msgs/msg/imu.hpp"
#include "trainros_imu_driver/imu_parser.hpp"

using namespace std::chrono_literals;
using trainros_imu_driver::ImuSample;
using trainros_imu_driver::kImuFrameSize;
using trainros_imu_driver::parse_imu33;
using trainros_imu_driver::rpy_to_quaternion;

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

class ImuDriverNode : public rclcpp::Node
{
public:
  ImuDriverNode() : Node("trainros_imu_driver")
  {
    port_ = declare_parameter<std::string>("port", "/dev/ttyUSB0");
    frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");
    baud_rate_ = declare_parameter<int>("baud_rate", 115200);
    reconnect_ms_ = declare_parameter<int>("reconnect_ms", 1000);
    diagnostics_period_ms_ = declare_parameter<int>("diagnostics_period_ms", 1000);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();
    publisher_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data", qos);
    diagnostics_publisher_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", rclcpp::QoS(10).reliable());
    poll_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    diagnostics_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_ = create_wall_timer(5ms, std::bind(&ImuDriverNode::poll_serial, this), poll_group_);
    diagnostics_timer_ = create_wall_timer(
      std::chrono::milliseconds(diagnostics_period_ms_),
      std::bind(&ImuDriverNode::publish_diagnostics, this),
      diagnostics_group_);

    RCLCPP_INFO(get_logger(), "IMU driver 已启动，端口：%s", port_.c_str());
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
      last_error_ = "打开串口失败";
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "IMU serial 打开失败：%s", port_.c_str());
      return;
    }
    last_error_.clear();
    RCLCPP_INFO(get_logger(), "IMU serial 已打开：%s", port_.c_str());
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
      buffer_.insert(buffer_.end(), tmp, tmp + n);
      parse_buffer();
      return;
    }
    if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      last_error_ = std::strerror(errno);
      RCLCPP_WARN(get_logger(), "IMU serial 读取失败，准备重连：%s", std::strerror(errno));
      serial_.close_port();
    }
  }

  void parse_buffer()
  {
    while (buffer_.size() >= kImuFrameSize) {
      auto it = std::search(buffer_.begin(), buffer_.end(), kHeader_, kHeader_ + 2);
      if (it == buffer_.end()) {
        buffer_.clear();
        return;
      }
      buffer_.erase(buffer_.begin(), it);
      if (buffer_.size() < kImuFrameSize) {
        return;
      }

      std::vector<uint8_t> frame(buffer_.begin(), buffer_.begin() + kImuFrameSize);
      buffer_.erase(buffer_.begin(), buffer_.begin() + kImuFrameSize);

      ImuSample sample;
      if (!parse_imu33(frame, sample)) {
        ++dropped_frames_;
        last_frame_valid_ = false;
        continue;
      }
      ++valid_frames_;
      last_frame_valid_ = true;
      publish_sample(sample);
    }
  }

  void publish_sample(const ImuSample & sample)
  {
    const auto quat = rpy_to_quaternion(sample.roll, sample.pitch, sample.yaw);

    sensor_msgs::msg::Imu msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.orientation.x = quat[0];
    msg.orientation.y = quat[1];
    msg.orientation.z = quat[2];
    msg.orientation.w = quat[3];
    msg.angular_velocity.x = sample.wx;
    msg.angular_velocity.y = sample.wy;
    msg.angular_velocity.z = sample.wz;
    msg.linear_acceleration.x = sample.ax;
    msg.linear_acceleration.y = sample.ay;
    msg.linear_acceleration.z = sample.az;
    publisher_->publish(msg);
  }

  void publish_diagnostics()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "trainros_imu_driver";
    status.hardware_id = port_;
    if (!serial_.is_open()) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "serial 未打开";
    } else if (valid_frames_ == 0) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "等待有效 IMU 数据帧";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "IMU 数据正常";
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
    add_key_value(status.values, "last_frame_valid", last_frame_valid_ ? "true" : "false");

    array.status.push_back(status);
    diagnostics_publisher_->publish(array);
  }

  static constexpr uint8_t kHeader_[2] = {0x55, 0x51};

  std::string port_;
  std::string frame_id_;
  int baud_rate_ = 115200;
  int reconnect_ms_ = 1000;
  int diagnostics_period_ms_ = 1000;
  uint64_t valid_frames_ = 0;
  uint64_t dropped_frames_ = 0;
  uint64_t reconnect_count_ = 0;
  bool last_frame_valid_ = false;
  std::string last_error_ = "none";
  rclcpp::Time last_reconnect_;
  rclcpp::Time last_receive_time_;
  std::vector<uint8_t> buffer_;
  SerialPort serial_;
  std::mutex mutex_;
  rclcpp::CallbackGroup::SharedPtr poll_group_;
  rclcpp::CallbackGroup::SharedPtr diagnostics_group_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImuDriverNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
