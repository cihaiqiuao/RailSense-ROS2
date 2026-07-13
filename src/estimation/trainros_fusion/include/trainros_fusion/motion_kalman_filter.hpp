#pragma once

#include <algorithm>
#include <array>
#include <cstdint>

namespace trainros_fusion
{

struct MotionKalmanConfig
{
  double output_rate_hz = 50.0;
  double process_noise_position = 0.1;
  double process_noise_speed = 0.2;
  double process_noise_acceleration = 1.0;
  double laser_distance_measurement_noise = 0.05;
  double gps_speed_measurement_noise = 0.5;
  double imu_acceleration_measurement_noise = 0.2;
};

struct MotionKalmanState
{
  bool initialized = false;
  bool has_laser_measurement = false;
  double position = 0.0;
  double speed = 0.0;
  double acceleration = 0.0;
  double raw_gps_speed = 0.0;
  double raw_laser_distance = 0.0;
  uint64_t imu_update_count = 0;
  uint64_t gps_update_count = 0;
  uint64_t laser_update_count = 0;
};

class MotionKalmanFilter
{
public:
  explicit MotionKalmanFilter(const MotionKalmanConfig & config = {})
  : config_(config)
  {
  }

  void update_config(const MotionKalmanConfig & config)
  {
    config_ = config;
  }

  const MotionKalmanState & state() const
  {
    return state_;
  }

  void update_imu(const double measured_acceleration, const double stamp_sec)
  {
    if (!state_.initialized) {
      state_.initialized = true;
      state_.acceleration = measured_acceleration;
      last_stamp_sec_ = stamp_sec;
    } else {
      double dt = stamp_sec - last_stamp_sec_;
      if (dt <= 0.0 || dt > 1.0) {
        dt = 1.0 / std::max(1.0, config_.output_rate_hz);
      }
      predict(dt);
      last_stamp_sec_ = stamp_sec;
    }

    update_scalar({0.0, 0.0, 1.0}, measured_acceleration, config_.imu_acceleration_measurement_noise);
    ++state_.imu_update_count;
  }

  void update_gps_speed(const double measured_speed)
  {
    state_.raw_gps_speed = measured_speed;
    if (!state_.initialized) {
      state_.initialized = true;
      state_.speed = measured_speed;
    }

    update_scalar({0.0, 1.0, 0.0}, measured_speed, config_.gps_speed_measurement_noise);
    ++state_.gps_update_count;
  }

  void update_laser_distance(const double measured_distance)
  {
    state_.raw_laser_distance = measured_distance;
    if (!state_.initialized) {
      state_.initialized = true;
      state_.position = measured_distance;
    }
    if (!state_.has_laser_measurement) {
      state_.has_laser_measurement = true;
      state_.position = measured_distance;
      covariance_[0][0] = config_.laser_distance_measurement_noise;
    }

    update_scalar({1.0, 0.0, 0.0}, measured_distance, config_.laser_distance_measurement_noise);
    ++state_.laser_update_count;
  }

  void predict(const double dt)
  {
    if (dt <= 0.0) {
      return;
    }

    state_.position += state_.speed * dt + 0.5 * state_.acceleration * dt * dt;
    state_.speed += state_.acceleration * dt;

    const std::array<std::array<double, 3>, 3> f {{
      {{1.0, dt, 0.5 * dt * dt}},
      {{0.0, 1.0, dt}},
      {{0.0, 0.0, 1.0}},
    }};
    std::array<std::array<double, 3>, 3> predicted {};
    for (std::size_t r = 0; r < 3; ++r) {
      for (std::size_t c = 0; c < 3; ++c) {
        for (std::size_t i = 0; i < 3; ++i) {
          for (std::size_t j = 0; j < 3; ++j) {
            predicted[r][c] += f[r][i] * covariance_[i][j] * f[c][j];
          }
        }
      }
    }
    predicted[0][0] += config_.process_noise_position * dt;
    predicted[1][1] += config_.process_noise_speed * dt;
    predicted[2][2] += config_.process_noise_acceleration * dt;
    covariance_ = predicted;
  }

private:
  void update_scalar(
    const std::array<double, 3> & h,
    const double measurement,
    const double measurement_noise)
  {
    std::array<double, 3> ph {};
    for (std::size_t r = 0; r < 3; ++r) {
      for (std::size_t c = 0; c < 3; ++c) {
        ph[r] += covariance_[r][c] * h[c];
      }
    }

    double predicted_measurement =
      h[0] * state_.position + h[1] * state_.speed + h[2] * state_.acceleration;
    double s = measurement_noise;
    for (std::size_t i = 0; i < 3; ++i) {
      s += h[i] * ph[i];
    }
    if (s <= 0.0) {
      return;
    }

    const double innovation = measurement - predicted_measurement;
    std::array<double, 3> k {};
    for (std::size_t i = 0; i < 3; ++i) {
      k[i] = ph[i] / s;
    }

    state_.position += k[0] * innovation;
    state_.speed += k[1] * innovation;
    state_.acceleration += k[2] * innovation;

    std::array<std::array<double, 3>, 3> updated {};
    for (std::size_t r = 0; r < 3; ++r) {
      for (std::size_t c = 0; c < 3; ++c) {
        updated[r][c] = covariance_[r][c] - k[r] * ph[c];
      }
    }
    covariance_ = updated;
  }

  MotionKalmanConfig config_;
  MotionKalmanState state_;
  double last_stamp_sec_ = 0.0;
  std::array<std::array<double, 3>, 3> covariance_ {{
    {{10.0, 0.0, 0.0}},
    {{0.0, 10.0, 0.0}},
    {{0.0, 0.0, 10.0}},
  }};
};

}  // namespace trainros_fusion
