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

#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <boost/algorithm/string.hpp>
#include <gtest/gtest.h>
#include <opensw/base64.h>
#include <opensw/protocol.h>
#include <spdlog/spdlog.h>

using namespace spdlog;

class ScanDataTest :public ::testing::TestWithParam<std::string> {};

TEST_P(ScanDataTest, ParseTest) {
    set_level(level::level_enum::info);

    auto file = GetParam();
    std::string filepath = std::string(DATA_DIR) + file;
    debug("Opening test data file: {}", filepath);

    std::ifstream data_file(filepath);
    std::string line;
    while (std::getline(data_file, line) && (line.empty() || line[0] == '#')) {
      // skip blank or comment lines
    }

    ASSERT_FALSE(line.empty());

    auto points = opensw::parseLaserPoints(line);

    std::vector<opensw::LaserPoint> expected_points;
    while (std::getline(data_file, line)) {
        // skip blank or comment lines
        if (line.empty() || line[0] == '#') {
            continue;
        }

        std::vector<std::string> values;
        boost::split(values, line, boost::is_any_of(" "));
        ASSERT_EQ(3, values.size());
        opensw::LaserPoint point;
        point.distance = std::stof(values[0]);
        point.angle = std::stof(values[1]);
        point.valid = std::stoi(values[2]);
        expected_points.push_back(point);
    }

    debug("Found {} expected points", expected_points.size());


    std::reverse(expected_points.begin(), expected_points.end());

    ASSERT_EQ(expected_points.size(), points.size());
    for (size_t i = 0; i < points.size(); i++) {

        uint8_t* d0 = reinterpret_cast<uint8_t*>(&expected_points[i].distance);
        uint8_t* d1 = reinterpret_cast<uint8_t*>(&points[i].distance);

        uint8_t* a0 = reinterpret_cast<uint8_t*>(&expected_points[i].angle);
        uint8_t* a1 = reinterpret_cast<uint8_t*>(&points[i].angle);

        trace("[{}] distance: 0x{:X} 0x{:X} 0x{:X} 0x{:X} ({}),  0x{:X} 0x{:X} 0x{:X} 0x{:X} ({})", i,
            d0[0], d0[1], d0[2], d0[3], expected_points[i].distance,
            d1[0], d1[1], d1[2], d1[3], points[i].distance);
        trace("[{}] angle: 0x{:X} 0x{:X} 0x{:X} 0x{:X} ({}),  0x{:X} 0x{:X} 0x{:X} 0x{:X} ({})", i,
            a0[0], a0[1], a0[2], a0[3], expected_points[i].angle,
            a1[0], a1[1], a1[2], a1[3], points[i].angle);

        ASSERT_FLOAT_EQ(expected_points[i].distance, points[i].distance);
        ASSERT_FLOAT_EQ(expected_points[i].angle, points[i].angle);
        ASSERT_EQ(expected_points[i].valid, points[i].valid);
    }
}

INSTANTIATE_TEST_CASE_P(
    ScanDataTests,
    ScanDataTest,
    ::testing::Values(
        "laser_scan_00.txt",
        "laser_scan_01.txt",
        "laser_scan_02.txt",
        "laser_scan_03.txt",
        "laser_scan_04.txt",
        "laser_scan_05.txt",
        "laser_scan_06.txt",
        "laser_scan_07.txt",
        "laser_scan_08.txt",
        "laser_scan_09.txt",
        "laser_scan_10.txt",
        "laser_scan_11.txt",
        "laser_scan_12.txt",
        "laser_scan_13.txt",
        "laser_scan_14.txt",
        "laser_scan_15.txt",
        "laser_scan_16.txt",
        "laser_scan_17.txt",
        "laser_scan_18.txt",
        "laser_scan_19.txt",
        "laser_scan_20.txt",
        "laser_scan_21.txt",
        "laser_scan_22.txt",
        "laser_scan_23.txt"
    ));
