#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <numeric>
#include <string>

namespace trainros_stability_evaluator
{

struct StabilityConfig
{
  double warning_threshold = 70.0;
  double alarm_threshold = 40.0;
  int window_size = 250;
  int min_window_samples = 50;
  double sample_rate_hz = 50.0;
  double psd_min_hz = 0.5;
  double psd_max_hz = 20.0;
  double rms_weight = 1.0;
  double peak_weight = 0.3;
  double psd_weight = 0.2;
  double tsi_alarm_value = 5.0;
};

struct StabilityMetrics
{
  float rms_acceleration = 0.0F;
  float peak_acceleration = 0.0F;
  float psd_value = 0.0F;
  float tsi = 0.0F;
  float score = 100.0F;
  bool warning = false;
  bool alarm = false;
  std::string level = "normal";
  std::size_t sample_count = 0;
};

class StabilityWindowEvaluator
{
public:
  explicit StabilityWindowEvaluator(const StabilityConfig & config)
  : config_(sanitize_config(config))
  {
  }

  void set_config(const StabilityConfig & config)
  {
    config_ = sanitize_config(config);
    trim_window();
  }

  const StabilityConfig & config() const
  {
    return config_;
  }

  void add_sample(const double acceleration)
  {
    window_.push_back(acceleration);
    trim_window();
  }

  StabilityMetrics evaluate() const
  {
    StabilityMetrics metrics;
    metrics.sample_count = window_.size();
    if (window_.empty()) {
      return metrics;
    }

    double square_sum = 0.0;
    double peak = 0.0;
    for (const double value : window_) {
      square_sum += value * value;
      peak = std::max(peak, std::abs(value));
    }

    const double rms = std::sqrt(square_sum / static_cast<double>(window_.size()));
    const double psd = compute_band_power();
    const double tsi = config_.rms_weight * rms + config_.peak_weight * peak +
      config_.psd_weight * std::sqrt(std::max(0.0, psd));
    const double score = clamp(100.0 - tsi / config_.tsi_alarm_value * 100.0, 0.0, 100.0);

    metrics.rms_acceleration = static_cast<float>(rms);
    metrics.peak_acceleration = static_cast<float>(peak);
    metrics.psd_value = static_cast<float>(psd);
    metrics.tsi = static_cast<float>(tsi);
    metrics.score = static_cast<float>(score);
    metrics.warning = score < config_.warning_threshold;
    metrics.alarm = score < config_.alarm_threshold;
    if (window_.size() < static_cast<std::size_t>(config_.min_window_samples)) {
      metrics.alarm = false;
    }
    metrics.level = metrics.alarm ? "alarm" : (metrics.warning ? "warning" : "normal");
    return metrics;
  }

private:
  static double clamp(const double value, const double low, const double high)
  {
    return std::max(low, std::min(value, high));
  }

  static StabilityConfig sanitize_config(StabilityConfig config)
  {
    config.window_size = std::max(1, config.window_size);
    config.min_window_samples = std::max(1, config.min_window_samples);
    config.sample_rate_hz = std::max(1.0, config.sample_rate_hz);
    config.psd_min_hz = std::max(0.0, config.psd_min_hz);
    config.psd_max_hz = std::max(config.psd_min_hz, config.psd_max_hz);
    config.tsi_alarm_value = std::max(0.001, config.tsi_alarm_value);
    config.rms_weight = std::max(0.0, config.rms_weight);
    config.peak_weight = std::max(0.0, config.peak_weight);
    config.psd_weight = std::max(0.0, config.psd_weight);
    return config;
  }

  void trim_window()
  {
    while (window_.size() > static_cast<std::size_t>(config_.window_size)) {
      window_.pop_front();
    }
  }

  double compute_band_power() const
  {
    const std::size_t n = window_.size();
    if (n < 2) {
      return 0.0;
    }

    constexpr double kPi = 3.14159265358979323846;
    const double mean =
      std::accumulate(window_.begin(), window_.end(), 0.0) / static_cast<double>(n);
    double band_power = 0.0;
    std::size_t bin_count = 0;

    for (std::size_t k = 1; k <= n / 2; ++k) {
      const double frequency = static_cast<double>(k) * config_.sample_rate_hz / static_cast<double>(n);
      if (frequency < config_.psd_min_hz || frequency > config_.psd_max_hz) {
        continue;
      }

      double real = 0.0;
      double imag = 0.0;
      for (std::size_t i = 0; i < n; ++i) {
        const double centered = window_[i] - mean;
        const double angle = 2.0 * kPi * static_cast<double>(k * i) / static_cast<double>(n);
        real += centered * std::cos(angle);
        imag -= centered * std::sin(angle);
      }
      band_power += (real * real + imag * imag) / static_cast<double>(n * n);
      ++bin_count;
    }

    return bin_count == 0 ? 0.0 : band_power / static_cast<double>(bin_count);
  }

  StabilityConfig config_;
  std::deque<double> window_;
};

}  // namespace trainros_stability_evaluator
