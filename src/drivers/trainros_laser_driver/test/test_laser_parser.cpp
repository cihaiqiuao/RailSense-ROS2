#include <cstdint>
#include <vector>

#include "gtest/gtest.h"
#include "trainros_laser_driver/laser_parser.hpp"

TEST(LaserParser, ParsesValidFrame)
{
  std::vector<uint8_t> frame(trainros_laser_driver::kLaserFrameSize, 0);
  frame[0] = 0xAA;
  frame[9] = 0xD2;
  frame[10] = 0x04;

  double distance_m = 0.0;
  ASSERT_TRUE(trainros_laser_driver::parse_laser12(frame, distance_m));
  EXPECT_NEAR(distance_m, 1.234, 1e-6);
}

TEST(LaserParser, RejectsShortFrame)
{
  std::vector<uint8_t> frame(11, 0);
  frame[0] = 0xAA;

  double distance_m = 0.0;
  EXPECT_FALSE(trainros_laser_driver::parse_laser12(frame, distance_m));
}

TEST(LaserParser, RejectsBadHeader)
{
  std::vector<uint8_t> frame(trainros_laser_driver::kLaserFrameSize, 0);
  frame[0] = 0xAB;

  double distance_m = 0.0;
  EXPECT_FALSE(trainros_laser_driver::parse_laser12(frame, distance_m));
}
