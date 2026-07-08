#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace trainros_laser_driver
{

constexpr std::size_t kLaserFrameSize = 12;

inline uint16_t read_u16_le(const std::vector<uint8_t> & data, const std::size_t offset)
{
  return static_cast<uint16_t>(data[offset]) |
    (static_cast<uint16_t>(data[offset + 1]) << 8);
}

inline bool parse_laser12(const std::vector<uint8_t> & frame, double & distance_m)
{
  if (frame.size() != kLaserFrameSize || frame[0] != 0xAA) {
    return false;
  }
  distance_m = static_cast<double>(read_u16_le(frame, 9)) / 1000.0;
  return true;
}

}  // namespace trainros_laser_driver
