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

// commands
const std::string CMD_GET_IMU_IN_ROBOT_COORDINATE = "getimuinrobotcoordinate";
const std::string CMD_GET_KNOWN_AREA = "getknownarea";
const std::string CMD_GET_LASER_SCAN = "getlaserscan";
const std::string CMD_GET_LOCATION = "getlocation";
const std::string CMD_GET_MAP_DATA = "getmapdata";
const std::string CMD_GET_POSE = "getpose";
const std::string CMD_GET_SPD_VERSION = "getsdpversion";
const std::string CMD_GET_SYSTEM_RESOURCE = "getsystemresource";


std::vector<uint8_t> inflate(const std::string& base64_data);
std::vector<LaserPoint> parseLaserPoints(const std::string& base64_data, bool fill_gaps=false);

}  // namespace rpad
