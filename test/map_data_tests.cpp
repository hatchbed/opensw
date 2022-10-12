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
#include <opensw/protocol.h>
#include <opensw/msg/GetMapDataResponse.h>
#include <spdlog/spdlog.h>

using namespace spdlog;

class MapDataTest :public ::testing::TestWithParam<std::string> {};

TEST_P(MapDataTest, ParseGetMapDataResponse) {
    set_level(level::level_enum::trace);

    auto file = GetParam();
    std::string filepath = std::string(DATA_DIR) + file;
    debug("Opening test data file: {}", filepath);

    std::ifstream data_file(filepath);
    std::string line;
    while (std::getline(data_file, line) && (line.empty() || line[0] == '#')) {
      // skip blank or comment lines
    }

    ASSERT_FALSE(line.empty());

    auto response = opensw::msg::GetMapDataResponse::fromJson(line);
    ASSERT_TRUE(response);

    auto map_data = opensw::inflate(response->result.map_data);
}

INSTANTIATE_TEST_CASE_P(
    MapDataTests,
    MapDataTest,
    ::testing::Values(
        "getmapdata_01.txt"
    ));
