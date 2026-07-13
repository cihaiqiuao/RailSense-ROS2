#include "gtest/gtest.h"
#include "trainros_fusion/motion_kalman_filter.hpp"

TEST(MotionKalmanFilter, ImuAccelerationInitializesAndUpdates)
{
  trainros_fusion::MotionKalmanFilter filter;
  filter.update_imu(2.0, 1.0);

  const auto & state = filter.state();
  EXPECT_TRUE(state.initialized);
  EXPECT_NEAR(state.acceleration, 2.0, 0.1);
  EXPECT_EQ(state.imu_update_count, 1U);
}

TEST(MotionKalmanFilter, GpsSpeedCorrectsSpeed)
{
  trainros_fusion::MotionKalmanFilter filter;
  filter.update_imu(0.0, 1.0);
  filter.update_gps_speed(5.0);

  const auto & state = filter.state();
  EXPECT_GT(state.speed, 0.0);
  EXPECT_LT(state.speed, 5.1);
  EXPECT_EQ(state.gps_update_count, 1U);
  EXPECT_NEAR(state.raw_gps_speed, 5.0, 1e-6);
}

TEST(MotionKalmanFilter, LaserDistanceCorrectsPosition)
{
  trainros_fusion::MotionKalmanFilter filter;
  filter.update_laser_distance(1.234);

  const auto & state = filter.state();
  EXPECT_TRUE(state.initialized);
  EXPECT_TRUE(state.has_laser_measurement);
  EXPECT_NEAR(state.position, 1.234, 1e-3);
  EXPECT_NEAR(state.raw_laser_distance, 1.234, 1e-6);
  EXPECT_EQ(state.laser_update_count, 1U);
}

TEST(MotionKalmanFilter, PredictContinuesWhenGpsMissing)
{
  trainros_fusion::MotionKalmanFilter filter;
  filter.update_imu(1.0, 1.0);
  filter.update_gps_speed(2.0);
  const double speed_before = filter.state().speed;

  filter.update_imu(1.0, 1.2);

  EXPECT_GT(filter.state().speed, speed_before);
  EXPECT_GT(filter.state().position, 0.0);
}

TEST(MotionKalmanFilter, RejectsInvalidPredictionDt)
{
  trainros_fusion::MotionKalmanFilter filter;
  filter.update_laser_distance(2.0);
  filter.predict(-1.0);
  EXPECT_NEAR(filter.state().position, 2.0, 1e-6);
}
