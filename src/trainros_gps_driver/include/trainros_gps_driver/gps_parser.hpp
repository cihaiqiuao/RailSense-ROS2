#pragma once

#include <cstdlib>
#include <sstream>
#include <string>
#include <vector>

namespace trainros_gps_driver
{

struct RmcSample
{
  bool valid_fix = false;
  double latitude = 0.0;
  double longitude = 0.0;
};

inline std::vector<std::string> split_csv(const std::string & line)
{
  std::vector<std::string> fields;
  std::stringstream ss(line);
  std::string item;
  while (std::getline(ss, item, ',')) {
    fields.push_back(item);
  }
  return fields;
}

inline bool dm_to_deg(const std::string & dm, double & deg)
{
  if (dm.empty()) {
    return false;
  }
  char * end = nullptr;
  const double value = std::strtod(dm.c_str(), &end);
  if (end == dm.c_str()) {
    return false;
  }
  const int degree = static_cast<int>(value / 100.0);
  const double minute = value - static_cast<double>(degree) * 100.0;
  deg = static_cast<double>(degree) + minute / 60.0;
  return true;
}

inline bool parse_rmc(const std::string & raw_line, RmcSample & out)
{
  std::string line = raw_line;
  const auto star = line.find('*');
  if (star != std::string::npos) {
    line = line.substr(0, star);
  }
  if (line.rfind("$GNRMC", 0) != 0 && line.rfind("$GPRMC", 0) != 0) {
    return false;
  }

  const auto fields = split_csv(line);
  if (fields.size() < 7) {
    return false;
  }

  out = RmcSample{};
  out.valid_fix = fields[2] == "A";
  if (!out.valid_fix) {
    return true;
  }

  if (!dm_to_deg(fields[3], out.latitude) || !dm_to_deg(fields[5], out.longitude)) {
    out.valid_fix = false;
    return true;
  }
  if (fields[4] == "S") {
    out.latitude = -out.latitude;
  }
  if (fields[6] == "W") {
    out.longitude = -out.longitude;
  }
  return true;
}

}  // namespace trainros_gps_driver
