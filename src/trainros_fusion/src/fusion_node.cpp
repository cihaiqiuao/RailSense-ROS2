#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "trainros_interfaces/msg/coupler.hpp"
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

void quaternion_to_rpy(
  const double x, const double y, const double z, const double w,
  double & roll, double & pitch, double & yaw)
{
  const double sinr_cosp = 2.0 * (w * x + y * z);
  const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
  roll = std::atan2(sinr_cosp, cosr_cosp);

  const double sinp = 2.0 * (w * y - z * x);
  if (std::abs(sinp) >= 1.0) {
    pitch = std::copysign(3.14159265358979323846 / 2.0, sinp);
  } else {
    pitch = std::asin(sinp);
  }

  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  yaw = std::atan2(siny_cosp, cosy_cosp);
}
}  // namespace

class FusionNode : public rclcpp::Node
{
public:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Imu,
    sensor_msgs::msg::LaserScan>;

  FusionNode() : Node("trainros_fusion")
  {
    output_rate_hz_ = declare_parameter<double>("output_rate_hz", 50.0);
    frame_id_ = declare_parameter<std::string>("frame_id", "base_link");
    sync_queue_size_ = declare_parameter<int>("sync_queue_size", 20);
    sync_slop_ms_ = declare_parameter<int>("sync_slop_ms", 50);
    diagnostics_period_ms_ = declare_parameter<int>("diagnostics_period_ms", 1000);
    gps_timeout_ms_ = declare_parameter<int>("gps_timeout_ms", 1000);
    coupler_timeout_ms_ = declare_parameter<int>("coupler_timeout_ms", 1000);
    sensor_timeout_ms_ = declare_parameter<int>("sensor_timeout_ms", 500);
    enable_kalman_filter_ = declare_parameter<bool>("enable_kalman_filter", true);
    process_noise_speed_ = declare_parameter<double>("process_noise_speed", 0.2);
    process_noise_acceleration_ = declare_parameter<double>("process_noise_acceleration", 1.0);
    gps_speed_measurement_noise_ = declare_parameter<double>("gps_speed_measurement_noise", 0.5);
    imu_acceleration_measurement_noise_ = declare_parameter<double>("imu_acceleration_measurement_noise", 0.2);

    publish_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    diagnostics_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    gps_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    coupler_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    publisher_ = create_publisher<trainros_interfaces::msg::TrainState>(
      "/train_state", rclcpp::QoS(10).reliable());
    diagnostics_publisher_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", rclcpp::QoS(10).reliable());

    imu_sub_.subscribe(this, "/imu/data", rmw_qos_profile_sensor_data);
    laser_sub_.subscribe(this, "/laser/scan", rmw_qos_profile_sensor_data);

    synchronizer_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(sync_queue_size_), imu_sub_, laser_sub_);
    synchronizer_->setMaxIntervalDuration(
      rclcpp::Duration::from_seconds(static_cast<double>(sync_slop_ms_) / 1000.0));
    synchronizer_->registerCallback(
      std::bind(
        &FusionNode::on_synced_sensors,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    rclcpp::SubscriptionOptions gps_options;
    gps_options.callback_group = gps_group_;
    gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gps/fix",
      rclcpp::QoS(10).reliable(),
      std::bind(&FusionNode::on_gps, this, std::placeholders::_1),
      gps_options);

    rclcpp::SubscriptionOptions coupler_options;
    coupler_options.callback_group = coupler_group_;
    coupler_sub_ = create_subscription<trainros_interfaces::msg::Coupler>(
      "/coupler_detection",
      rclcpp::QoS(10).reliable(),
      std::bind(&FusionNode::on_coupler, this, std::placeholders::_1),
      coupler_options);

    const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / output_rate_hz_));
    timer_ = create_wall_timer(period, std::bind(&FusionNode::publish_state, this), publish_group_);
    diagnostics_timer_ = create_wall_timer(
      std::chrono::milliseconds(diagnostics_period_ms_),
      std::bind(&FusionNode::publish_diagnostics, this),
      diagnostics_group_);

    RCLCPP_INFO(
      get_logger(),
      "Fusion started, output %.1f Hz, IMU/Laser sync window %d ms",
      output_rate_hz_,
      sync_slop_ms_);
  }

private:
  void on_synced_sensors(
    const sensor_msgs::msg::Imu::ConstSharedPtr imu,
    const sensor_msgs::msg::LaserScan::ConstSharedPtr laser)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++imu_laser_sync_count_;
    last_sync_time_ = now();
    const double raw_acceleration = imu->linear_acceleration.x;
    update_motion_filter_with_imu(*imu, raw_acceleration);
    acceleration_ = enable_kalman_filter_ ? static_cast<float>(kf_acceleration_) :
      static_cast<float>(raw_acceleration);
    quaternion_to_rpy(
      imu->orientation.x, imu->orientation.y, imu->orientation.z, imu->orientation.w,
      roll_, pitch_, yaw_);
    last_imu_stamp_ = imu->header.stamp;
    has_imu_ = true;

    if (!laser->ranges.empty() && std::isfinite(laser->ranges.front())) {
      laser_distance_ = laser->ranges.front();
      last_laser_stamp_ = laser->header.stamp;
      has_laser_ = true;
    }
  }

  void on_gps(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (has_gps_) {
      const rclcpp::Time current(msg->header.stamp);
      const double dt = (current - last_gps_stamp_).seconds();
      if (dt > 0.001) {
        const double d_lat = (msg->latitude - last_lat_) * 111320.0;
        const double d_lon = (msg->longitude - last_lon_) * 111320.0 *
          std::cos(msg->latitude * 3.14159265358979323846 / 180.0);
        const double gps_speed = std::sqrt(d_lat * d_lat + d_lon * d_lon) / dt;
        raw_gps_speed_ = gps_speed;
        if (enable_kalman_filter_) {
          update_motion_filter_with_gps(gps_speed);
          speed_ = static_cast<float>(kf_speed_);
          acceleration_ = static_cast<float>(kf_acceleration_);
        } else {
          speed_ = static_cast<float>(gps_speed);
        }
      }
    }

    last_lat_ = msg->latitude;
    last_lon_ = msg->longitude;
    last_gps_stamp_ = msg->header.stamp;
    last_gps_receive_time_ = now();
    has_gps_ = true;
  }

  void on_coupler(const trainros_interfaces::msg::Coupler::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    coupler_status_ = msg->status;
    last_coupler_stamp_ = msg->header.stamp;
    last_coupler_receive_time_ = now();
    has_coupler_ = true;
  }

  void predict_motion_filter(const double dt)
  {
    if (dt <= 0.0) {
      return;
    }

    kf_speed_ += kf_acceleration_ * dt;

    const double p00 = kf_p00_ + dt * (kf_p10_ + kf_p01_) + dt * dt * kf_p11_ +
      process_noise_speed_ * dt;
    const double p01 = kf_p01_ + dt * kf_p11_;
    const double p10 = kf_p10_ + dt * kf_p11_;
    const double p11 = kf_p11_ + process_noise_acceleration_ * dt;
    kf_p00_ = p00;
    kf_p01_ = p01;
    kf_p10_ = p10;
    kf_p11_ = p11;
  }

  void update_motion_filter_with_imu(
    const sensor_msgs::msg::Imu & imu,
    const double measured_acceleration)
  {
    if (!enable_kalman_filter_) {
      return;
    }

    const rclcpp::Time stamp(imu.header.stamp);
    if (!kf_initialized_) {
      kf_initialized_ = true;
      kf_acceleration_ = measured_acceleration;
      last_filter_stamp_ = stamp;
    } else {
      double dt = (stamp - last_filter_stamp_).seconds();
      if (dt <= 0.0 || dt > 1.0) {
        dt = 1.0 / output_rate_hz_;
      }
      predict_motion_filter(dt);
      last_filter_stamp_ = stamp;
    }

    const double innovation = measured_acceleration - kf_acceleration_;
    const double s = kf_p11_ + imu_acceleration_measurement_noise_;
    if (s <= 0.0) {
      return;
    }
    const double k0 = kf_p01_ / s;
    const double k1 = kf_p11_ / s;
    kf_speed_ += k0 * innovation;
    kf_acceleration_ += k1 * innovation;

    const double p00 = kf_p00_ - k0 * kf_p10_;
    const double p01 = kf_p01_ - k0 * kf_p11_;
    const double p10 = kf_p10_ - k1 * kf_p10_;
    const double p11 = kf_p11_ - k1 * kf_p11_;
    kf_p00_ = p00;
    kf_p01_ = p01;
    kf_p10_ = p10;
    kf_p11_ = p11;
    ++kf_imu_update_count_;
  }

  void update_motion_filter_with_gps(const double measured_speed)
  {
    if (!enable_kalman_filter_) {
      return;
    }
    if (!kf_initialized_) {
      kf_initialized_ = true;
      kf_speed_ = measured_speed;
    }

    const double innovation = measured_speed - kf_speed_;
    const double s = kf_p00_ + gps_speed_measurement_noise_;
    if (s <= 0.0) {
      return;
    }
    const double k0 = kf_p00_ / s;
    const double k1 = kf_p10_ / s;
    kf_speed_ += k0 * innovation;
    kf_acceleration_ += k1 * innovation;

    const double p00 = kf_p00_ - k0 * kf_p00_;
    const double p01 = kf_p01_ - k0 * kf_p01_;
    const double p10 = kf_p10_ - k1 * kf_p00_;
    const double p11 = kf_p11_ - k1 * kf_p01_;
    kf_p00_ = p00;
    kf_p01_ = p01;
    kf_p10_ = p10;
    kf_p11_ = p11;
    ++kf_gps_update_count_;
  }

  double age_ms(const rclcpp::Time & stamp, const rclcpp::Time & current) const
  {
    if (stamp.nanoseconds() == 0) {
      return -1.0;
    }
    return (current - stamp).seconds() * 1000.0;
  }

  void publish_state()
  {
    trainros_interfaces::msg::TrainState msg;
    bool stale_sensor = false;
    double state_latency_ms = -1.0;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto current = now();
      const double imu_age_ms = age_ms(last_imu_stamp_, current);
      const double laser_age_ms = age_ms(last_laser_stamp_, current);
      stale_sensor = !has_imu_ || !has_laser_ ||
        imu_age_ms > static_cast<double>(sensor_timeout_ms_) ||
        laser_age_ms > static_cast<double>(sensor_timeout_ms_);
      if (stale_sensor) {
        ++dropped_due_to_stale_sensor_;
      }

      const rclcpp::Time source_stamp = last_laser_stamp_.nanoseconds() > last_imu_stamp_.nanoseconds() ?
        last_laser_stamp_ : last_imu_stamp_;
      state_latency_ms = age_ms(source_stamp, current);

      msg.header.stamp = current;
      msg.header.frame_id = frame_id_;
      msg.speed = speed_;
      msg.acceleration = has_imu_ ? acceleration_ : 0.0F;
      msg.roll = static_cast<float>(roll_);
      msg.pitch = static_cast<float>(pitch_);
      msg.yaw = static_cast<float>(yaw_);
      msg.laser_distance = has_laser_ ? laser_distance_ : 0.0F;
      msg.coupler_status = coupler_status_;
      last_state_latency_ms_ = state_latency_ms;
      ++published_count_;
    }

    publisher_->publish(msg);
  }

  void publish_diagnostics()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    diagnostic_msgs::msg::DiagnosticStatus status;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto current = now();
      array.header.stamp = current;
      status.name = "trainros_fusion";
      status.hardware_id = "software";

      const double gps_age = age_ms(last_gps_receive_time_, current);
      const double laser_age = age_ms(last_laser_stamp_, current);
      const double coupler_age = age_ms(last_coupler_receive_time_, current);

      if (imu_laser_sync_count_ == 0) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = "waiting_for_imu_laser_sync";
      } else if (gps_age < 0.0 || gps_age > static_cast<double>(gps_timeout_ms_)) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = "gps_stale";
      } else {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        status.message = "ok";
      }

      if (has_coupler_ && coupler_age > static_cast<double>(coupler_timeout_ms_)) {
        status.level = std::max<int8_t>(status.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
        status.message = "coupler_stale";
      }

      add_key_value(status.values, "imu_laser_sync_count", std::to_string(imu_laser_sync_count_));
      add_key_value(status.values, "published_count", std::to_string(published_count_));
      add_key_value(status.values, "has_imu", has_imu_ ? "true" : "false");
      add_key_value(status.values, "has_gps", has_gps_ ? "true" : "false");
      add_key_value(status.values, "has_laser", has_laser_ ? "true" : "false");
      add_key_value(status.values, "has_coupler", has_coupler_ ? "true" : "false");
      add_key_value(status.values, "gps_age_ms", std::to_string(gps_age));
      add_key_value(status.values, "laser_age_ms", std::to_string(laser_age));
      add_key_value(status.values, "coupler_age_ms", std::to_string(coupler_age));
      add_key_value(status.values, "state_latency_ms", std::to_string(last_state_latency_ms_));
      add_key_value(status.values, "kalman_enabled", enable_kalman_filter_ ? "true" : "false");
      add_key_value(status.values, "kalman_initialized", kf_initialized_ ? "true" : "false");
      add_key_value(status.values, "kalman_speed", std::to_string(kf_speed_));
      add_key_value(status.values, "kalman_acceleration", std::to_string(kf_acceleration_));
      add_key_value(status.values, "raw_gps_speed", std::to_string(raw_gps_speed_));
      add_key_value(status.values, "kalman_imu_update_count", std::to_string(kf_imu_update_count_));
      add_key_value(status.values, "kalman_gps_update_count", std::to_string(kf_gps_update_count_));
      add_key_value(
        status.values,
        "dropped_due_to_stale_sensor",
        std::to_string(dropped_due_to_stale_sensor_));
      add_key_value(
        status.values,
        "last_sync_time",
        last_sync_time_.nanoseconds() == 0 ? "never" : std::to_string(last_sync_time_.seconds()));
    }

    array.status.push_back(status);
    diagnostics_publisher_->publish(array);
  }

  double output_rate_hz_ = 50.0;
  std::string frame_id_;
  int sync_queue_size_ = 20;
  int sync_slop_ms_ = 50;
  int diagnostics_period_ms_ = 1000;
  int gps_timeout_ms_ = 1000;
  int coupler_timeout_ms_ = 1000;
  int sensor_timeout_ms_ = 500;
  bool enable_kalman_filter_ = true;
  double process_noise_speed_ = 0.2;
  double process_noise_acceleration_ = 1.0;
  double gps_speed_measurement_noise_ = 0.5;
  double imu_acceleration_measurement_noise_ = 0.2;
  float speed_ = 0.0F;
  float acceleration_ = 0.0F;
  double roll_ = 0.0;
  double pitch_ = 0.0;
  double yaw_ = 0.0;
  float laser_distance_ = 0.0F;
  double last_lat_ = 0.0;
  double last_lon_ = 0.0;
  bool has_imu_ = false;
  bool has_gps_ = false;
  bool has_laser_ = false;
  bool has_coupler_ = false;
  bool kf_initialized_ = false;
  uint64_t imu_laser_sync_count_ = 0;
  uint64_t published_count_ = 0;
  uint64_t dropped_due_to_stale_sensor_ = 0;
  uint64_t kf_imu_update_count_ = 0;
  uint64_t kf_gps_update_count_ = 0;
  double last_state_latency_ms_ = -1.0;
  double raw_gps_speed_ = 0.0;
  double kf_speed_ = 0.0;
  double kf_acceleration_ = 0.0;
  double kf_p00_ = 10.0;
  double kf_p01_ = 0.0;
  double kf_p10_ = 0.0;
  double kf_p11_ = 10.0;
  std::string coupler_status_ = "unknown";
  rclcpp::Time last_imu_stamp_;
  rclcpp::Time last_laser_stamp_;
  rclcpp::Time last_gps_stamp_;
  rclcpp::Time last_gps_receive_time_;
  rclcpp::Time last_coupler_stamp_;
  rclcpp::Time last_coupler_receive_time_;
  rclcpp::Time last_sync_time_;
  rclcpp::Time last_filter_stamp_;
  std::mutex mutex_;
  rclcpp::CallbackGroup::SharedPtr publish_group_;
  rclcpp::CallbackGroup::SharedPtr diagnostics_group_;
  rclcpp::CallbackGroup::SharedPtr gps_group_;
  rclcpp::CallbackGroup::SharedPtr coupler_group_;
  rclcpp::Publisher<trainros_interfaces::msg::TrainState>::SharedPtr publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub_;
  message_filters::Subscriber<sensor_msgs::msg::LaserScan> laser_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> synchronizer_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<trainros_interfaces::msg::Coupler>::SharedPtr coupler_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FusionNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
