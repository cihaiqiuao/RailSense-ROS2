#include <cmath>
#include <cstdint>
#include <vector>

#include "gtest/gtest.h"
#include "trainros_imu_driver/imu_parser.hpp"

namespace
{
void put_i16(std::vector<uint8_t> & frame, const std::size_t offset, const int16_t value)
{
  frame[offset] = static_cast<uint8_t>(value & 0xFF);
  frame[offset + 1] = static_cast<uint8_t>((static_cast<uint16_t>(value) >> 8) & 0xFF);
}

void fill_checksum(std::vector<uint8_t> & frame, const std::size_t offset)
{
  uint16_t sum = 0;
  for (std::size_t i = 0; i < 10; ++i) {
    sum += frame[offset + i];
  }
  frame[offset + 10] = static_cast<uint8_t>(sum & 0xFF);
}

std::vector<uint8_t> make_valid_frame()
{
  std::vector<uint8_t> frame(trainros_imu_driver::kImuFrameSize, 0);
  frame[0] = 0x55;
  frame[1] = 0x51;
  put_i16(frame, 2, 1024);
  put_i16(frame, 4, -1024);
  put_i16(frame, 6, 2048);
  fill_checksum(frame, 0);

  frame[11] = 0x55;
  frame[12] = 0x52;
  put_i16(frame, 13, 100);
  put_i16(frame, 15, -100);
  put_i16(frame, 17, 200);
  fill_checksum(frame, 11);

  frame[22] = 0x55;
  frame[23] = 0x53;
  put_i16(frame, 24, 1000);
  put_i16(frame, 26, -1000);
  put_i16(frame, 28, 500);
  fill_checksum(frame, 22);
  return frame;
}
}  // namespace

TEST(ImuParser, ParsesValidFrame)
{
  trainros_imu_driver::ImuSample sample;
  ASSERT_TRUE(trainros_imu_driver::parse_imu33(make_valid_frame(), sample));
  EXPECT_NEAR(sample.ax, 1024.0 * 16.0 * 9.8 / 32768.0, 1e-6);
  EXPECT_NEAR(sample.ay, -1024.0 * 16.0 * 9.8 / 32768.0, 1e-6);
  EXPECT_NEAR(sample.az, 2048.0 * 16.0 * 9.8 / 32768.0, 1e-6);
  EXPECT_NEAR(sample.roll, 1000.0 * 180.0 / 32768.0 * M_PI / 180.0, 1e-6);
}

TEST(ImuParser, RejectsBadHeader)
{
  auto frame = make_valid_frame();
  frame[12] = 0x50;
  trainros_imu_driver::ImuSample sample;
  EXPECT_FALSE(trainros_imu_driver::parse_imu33(frame, sample));
}

TEST(ImuParser, RejectsBadChecksum)
{
  auto frame = make_valid_frame();
  frame[10] ^= 0xFF;
  trainros_imu_driver::ImuSample sample;
  EXPECT_FALSE(trainros_imu_driver::parse_imu33(frame, sample));
}
