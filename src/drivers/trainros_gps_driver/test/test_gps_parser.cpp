#include "gtest/gtest.h"
#include "trainros_gps_driver/gps_parser.hpp"

TEST(GpsParser, ParsesValidRmc)
{
  trainros_gps_driver::RmcSample sample;
  ASSERT_TRUE(trainros_gps_driver::parse_rmc(
    "$GNRMC,092751.000,A,5321.6802,N,00630.3372,W,0.06,31.66,280511,,,A*43", sample));
  EXPECT_TRUE(sample.valid_fix);
  EXPECT_NEAR(sample.latitude, 53.3613367, 1e-6);
  EXPECT_NEAR(sample.longitude, -6.50562, 1e-6);
}

TEST(GpsParser, ParsesNoFixRmc)
{
  trainros_gps_driver::RmcSample sample;
  ASSERT_TRUE(trainros_gps_driver::parse_rmc(
    "$GPRMC,092751.000,V,,,,,,,280511,,,N*53", sample));
  EXPECT_FALSE(sample.valid_fix);
}

TEST(GpsParser, RejectsNonRmcSentence)
{
  trainros_gps_driver::RmcSample sample;
  EXPECT_FALSE(trainros_gps_driver::parse_rmc("$GNGGA,1,2,3", sample));
}
