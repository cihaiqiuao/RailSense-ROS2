#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace trainros_imu_driver
{

constexpr std::size_t kImuFrameSize = 33;

struct ImuSample
{
  double ax = 0.0;
  double ay = 0.0;
  double az = 0.0;
  double wx = 0.0;
  double wy = 0.0;
  double wz = 0.0;
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
};

inline int16_t read_i16_le(const std::vector<uint8_t> & data, const std::size_t offset)
{
  return static_cast<int16_t>(
    static_cast<uint16_t>(data[offset]) |
    (static_cast<uint16_t>(data[offset + 1]) << 8));
}

inline bool checksum_11(const std::vector<uint8_t> & data, const std::size_t offset)
{
  uint16_t sum = 0;
  for (std::size_t i = 0; i < 10; ++i) {
    sum += data[offset + i];
  }
  return static_cast<uint8_t>(sum & 0xFF) == data[offset + 10];
}

inline bool parse_imu33(const std::vector<uint8_t> & frame, ImuSample & out)
{
  if (frame.size() != kImuFrameSize) {
    return false;
  }
  if (frame[0] != 0x55 || frame[1] != 0x51) {
    return false;
  }
  if (frame[11] != 0x55 || frame[12] != 0x52) {
    return false;
  }
  if (frame[22] != 0x55 || frame[23] != 0x53) {
    return false;
  }
  if (!checksum_11(frame, 0) || !checksum_11(frame, 11) || !checksum_11(frame, 22)) {
    return false;
  }

  constexpr double g0 = 9.8;
  constexpr double accel_scale = 16.0 * g0 / 32768.0;
  constexpr double gyro_scale = 2000.0 / 32768.0;
  constexpr double angle_scale = 180.0 / 32768.0;
  constexpr double deg_to_rad = 3.14159265358979323846 / 180.0;

  out.ax = read_i16_le(frame, 2) * accel_scale;
  out.ay = read_i16_le(frame, 4) * accel_scale;
  out.az = read_i16_le(frame, 6) * accel_scale;
  out.wx = read_i16_le(frame, 13) * gyro_scale * deg_to_rad;
  out.wy = read_i16_le(frame, 15) * gyro_scale * deg_to_rad;
  out.wz = read_i16_le(frame, 17) * gyro_scale * deg_to_rad;
  out.roll = read_i16_le(frame, 24) * angle_scale * deg_to_rad;
  out.pitch = read_i16_le(frame, 26) * angle_scale * deg_to_rad;
  out.yaw = read_i16_le(frame, 28) * angle_scale * deg_to_rad;
  return true;
}

inline std::array<double, 4> rpy_to_quaternion(
  const double roll, const double pitch, const double yaw)
{
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);

  return {
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy,
    cr * cp * cy + sr * sp * sy};
}

}  // namespace trainros_imu_driver
