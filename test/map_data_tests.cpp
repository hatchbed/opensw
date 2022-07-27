#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <boost/algorithm/string.hpp>
#include <gtest/gtest.h>
#include <rpad/protocol.h>
#include <rpad/msg/GetMapDataResponse.h>
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

    auto response = rpad::msg::GetMapDataResponse::fromJson(line);
    ASSERT_TRUE(response);

    auto map_data = rpad::inflate(response->result.map_data);
}

INSTANTIATE_TEST_CASE_P(
    MapDataTests,
    MapDataTest,
    ::testing::Values(
        "getmapdata_01.txt"
    ));
