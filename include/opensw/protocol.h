/**
 * Copyright (c) 2022, Hatchbed
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <opensw/types.h>

namespace opensw {

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
const std::string CMD_GET_ROBOT_HEALTH = "getrobothealth";
const std::string CMD_GET_SPD_VERSION = "getsdpversion";
const std::string CMD_GET_SYSTEM_RESOURCE = "getsystemresource";

/**
 * Inflate encoded binary data.
 *
 * First the data is decoded from a base64 string and then it is processed to
 * decode from run length encoding.
 *
 * A 9 byte buffer in the decoded base64 data is assumed, which should contain
 * the specifics of the run length encoding:
 *
 *   [0] 'R'
 *   [1] 'L'
 *   [2] 'E'
 *   [3] sentinel byte 1 (usually 129)
 *   [4] sentinel byte 2 (usually 127)
 *   [5 - 8] length of inflated data (uint32)
 *
 * @param[in] base64_data  The base64 encoded data string.
 *
 * @returns The inflated data if successful, empty otherwise.
 */
std::vector<uint8_t> inflate(const std::string& base64_data);

/**
 * Parse laser points from base64 encoded laser point data.
 *
 * See documentation of `inflate` function.
 *
 * After the base64 encoded data is decoded and inflated, the memory layout of
 * the data is 12 bytes per point:
 *
 *    [0 - 3] distance (float)
 *    [4 - 7] angle (float)
 *    [8] valid (uint8)
 *    [9 - 11] padding
 *
 * @param[in] base64_data  The base64 encoded data string.
 * @param[in] fill_gaps  Whether or not to fill in any single point gaps with 'invalid' points.
 *
 * @returns Parsed laser points, sorted by ascending angle, if successful, empty otherwise.
 */
std::vector<LaserPoint> parseLaserPoints(const std::string& base64_data, bool fill_gaps=false);

}  // namespace opensw
