#include <cmath>

#include "gtest/gtest.h"
#include "trainros_stability_evaluator/stability_metrics.hpp"

namespace
{
trainros_stability_evaluator::StabilityConfig make_config()
{
  trainros_stability_evaluator::StabilityConfig config;
  config.window_size = 100;
  config.min_window_samples = 10;
  config.sample_rate_hz = 50.0;
  config.psd_min_hz = 0.5;
  config.psd_max_hz = 20.0;
  config.tsi_alarm_value = 5.0;
  return config;
}
}  // namespace

TEST(StabilityMetrics, ZeroInputIsNormal)
{
  trainros_stability_evaluator::StabilityWindowEvaluator evaluator(make_config());
  for (int i = 0; i < 50; ++i) {
    evaluator.add_sample(0.0);
  }

  const auto metrics = evaluator.evaluate();
  EXPECT_FLOAT_EQ(metrics.rms_acceleration, 0.0F);
  EXPECT_FLOAT_EQ(metrics.peak_acceleration, 0.0F);
  EXPECT_FLOAT_EQ(metrics.psd_value, 0.0F);
  EXPECT_FLOAT_EQ(metrics.tsi, 0.0F);
  EXPECT_FLOAT_EQ(metrics.score, 100.0F);
  EXPECT_EQ(metrics.level, "normal");
}

TEST(StabilityMetrics, ConstantAccelerationComputesRmsAndPeak)
{
  trainros_stability_evaluator::StabilityWindowEvaluator evaluator(make_config());
  for (int i = 0; i < 50; ++i) {
    evaluator.add_sample(2.0);
  }

  const auto metrics = evaluator.evaluate();
  EXPECT_NEAR(metrics.rms_acceleration, 2.0F, 1e-5F);
  EXPECT_NEAR(metrics.peak_acceleration, 2.0F, 1e-5F);
  EXPECT_NEAR(metrics.psd_value, 0.0F, 1e-5F);
  EXPECT_GT(metrics.tsi, 2.0F);
  EXPECT_LT(metrics.score, 100.0F);
}

TEST(StabilityMetrics, PulseInputKeepsPeak)
{
  auto config = make_config();
  config.window_size = 20;
  trainros_stability_evaluator::StabilityWindowEvaluator evaluator(config);
  for (int i = 0; i < 19; ++i) {
    evaluator.add_sample(0.0);
  }
  evaluator.add_sample(-5.0);

  const auto metrics = evaluator.evaluate();
  EXPECT_NEAR(metrics.peak_acceleration, 5.0F, 1e-5F);
  EXPECT_GT(metrics.rms_acceleration, 1.0F);
}

TEST(StabilityMetrics, SineInputProducesBandPower)
{
  auto config = make_config();
  config.window_size = 100;
  trainros_stability_evaluator::StabilityWindowEvaluator evaluator(config);
  for (int i = 0; i < 100; ++i) {
    evaluator.add_sample(std::sin(2.0 * M_PI * 5.0 * static_cast<double>(i) / config.sample_rate_hz));
  }

  const auto metrics = evaluator.evaluate();
  EXPECT_GT(metrics.psd_value, 0.0F);
  EXPECT_GT(metrics.tsi, metrics.rms_acceleration);
}

TEST(StabilityMetrics, WindowDropsOldSamples)
{
  auto config = make_config();
  config.window_size = 5;
  config.min_window_samples = 1;
  trainros_stability_evaluator::StabilityWindowEvaluator evaluator(config);
  evaluator.add_sample(10.0);
  for (int i = 0; i < 5; ++i) {
    evaluator.add_sample(1.0);
  }

  const auto metrics = evaluator.evaluate();
  EXPECT_EQ(metrics.sample_count, 5U);
  EXPECT_NEAR(metrics.peak_acceleration, 1.0F, 1e-5F);
}

TEST(StabilityMetrics, ConfigUpdateChangesAlarmDecision)
{
  auto config = make_config();
  config.min_window_samples = 1;
  config.tsi_alarm_value = 100.0;
  trainros_stability_evaluator::StabilityWindowEvaluator evaluator(config);
  evaluator.add_sample(2.0);
  EXPECT_FALSE(evaluator.evaluate().alarm);

  config.tsi_alarm_value = 1.0;
  evaluator.set_config(config);
  EXPECT_TRUE(evaluator.evaluate().alarm);
}
