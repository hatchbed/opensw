#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <rpad/types.h>

namespace rpad {

const std::string DELIM = "\n\r\n\r\n";
const uint8_t RLE_HEADER_SIZE = 9;
const float MAX_DISTANCE = 100.0f;
const float MAX_ANGLE = M_PI + 0.01;

std::vector<uint8_t> inflate(const std::string& base64_data);

std::vector<LaserPoint> parseLaserPoints(const std::string& base64_data, bool fill_gaps=false);

}  // namespace rpad
