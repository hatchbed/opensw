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

#include <opensw/protocol.h>

#include <algorithm>
#include <iostream>

#include <opensw/base64.h>
#include <opensw/logger.h>

using namespace spdlog;

namespace opensw {

std::vector<uint8_t> inflate(const std::string& base64_data) {
    debug("Inflating data ...");

    size_t data_size;
    unsigned char* data = base64_decode(
        reinterpret_cast<const unsigned char*>(base64_data.data()), base64_data.size(), &data_size);

    if (data_size > base64_data.size()) {
        error("Failed to decode base64 data. size = {}", data_size);
        free(data);
        return {};
    }

    if (default_logger()->level() == level::level_enum::trace) {
        trace("rle data:");
        for (size_t i = 0; i < data_size; i++) {
            trace("R[{}]: {:d}", i, data[i]);
        }
    }

    if (data_size < RLE_HEADER_SIZE) {
        error("Failed to find RLE header in data", data_size);
        free(data);
        return {};
    }

    // TODO check header signature

    uint8_t s1 = data[3]; // sentinel 1
    uint8_t s2 = data[4]; // sentinel 2
    uint32_t inflated_size;
    memcpy(&inflated_size, &data[5], sizeof(inflated_size));

    // TODO validate inflated size is reasonable

    debug("RLE header: signature({},{},{}) sentinel1(0x{:X}) sentinel2(0x{:X}) size({})",
        static_cast<char>(data[0]), static_cast<char>(data[1]), static_cast<char>(data[2]), s1, s2, inflated_size);

    std::vector<uint8_t> processed(inflated_size + 1);
    size_t processed_size = processed.size();


    size_t idx = 0;
    for (size_t i = RLE_HEADER_SIZE; i < data_size && idx < processed_size; i++) {
        if (data[i] == s1 && i + 2 < data_size) {
            // if it's not currently literal mode, and the byte is the start of an repeat section: (0x81, count, value)
            // determin what the count and value are
            uint8_t count = data[i + 1];
            uint8_t value = data[i + 2];

            // check if this is actually the escape sequence for entering literal mode
            if (count == 0x00 && value == s2) {
              trace("[{}:{}]: swap sentinel bytes", i, idx);
              std::swap(s1, s2);
            }
            else {
                trace("[{}:{}]: repeat {:d} (x{:d})", i, idx, value, count);
                // repeat the value
                for (uint8_t j = 0; j < count && idx < processed_size; j++) {
                    processed[idx++] = value;
                }
            }

            i += 2;
        }
        else {
          // copy the current byte to the destination, literally, without checking if it's the start of a repeat section
          processed[idx++] = data[i];
        }
    }

    processed.resize(idx);

    if (default_logger()->level() == level::level_enum::trace) {
        trace("inflated data:");
        for (size_t i = 0; i < processed.size(); i++) {
            trace("D[{}]: {:d}", i, processed[i]);
        }
    }

    if (processed.size() != inflated_size) {
        error("Failed to inflate data to the correct size: {} != {}", processed.size(), inflated_size);
        return {};
    }

    return processed;
}

std::vector<LaserPoint> parseLaserPoints(const std::string& base64_data, bool fill_gaps) {
    debug("Parsing laser points ...");

    auto data = inflate(base64_data);

    if (data.size() < 9) {
        warn("No points found in data.");
        return {};
    }

    std::vector<LaserPoint> points;
    points.reserve(data.size() / 12 + 10);

    int idx = data.size() - 1;
    LaserPoint point;
    int last_point_idx = data.size();
    bool recover = false;
    while (idx >= 8) {
        int start_idx = idx;

        uint8_t is_valid = data[idx];
        if (is_valid > 1) {
            if (!recover) {
                recover = true;
                idx = start_idx + 2;
                debug("Failed to find validity flag, resyncing ...");
            }
            else {
                debug("[{}] Scanning for validity flag ...", idx);
                idx = start_idx - 1;
            }
            continue;
        }
        else {
            point.valid = is_valid == 1;
        }

        memcpy(&point.angle, &data[idx - 4], sizeof(point.angle));
        if (!std::isfinite(point.angle) || point.angle < -MAX_ANGLE || point.angle > MAX_ANGLE || (point.angle != 0 && std::fabs(point.angle) < 0.0001)) {
            debug("[{}] Invalid angle: {}", idx - 4, point.angle);
            if (!recover) {
                recover = true;
                idx = start_idx + 2;
            }
            else {
                idx = start_idx - 1;
            }

            continue;
        }
        idx-=4;

        memcpy(&point.distance, &data[idx - 4], sizeof(point.distance));
        if (!std::isfinite(point.distance) || (point.distance != 100000 && (point.distance <= 0.001 || point.distance > MAX_DISTANCE))) {
            debug("[{}] Invalid distance: {}", idx - 4, point.distance);
            if (!recover) {
                recover = true;
                idx = start_idx + 2;
            }
            else {
                idx = start_idx - 1;
            }
            continue;
        }
        idx-=4;

        debug("[{}] Adding point {}: {} {} {:d}", idx, points.size(), point.distance, point.angle, point.valid);
        points.push_back(point);
        recover = false;

        if (last_point_idx - idx != 12) {
            warn("Unexpected byte gap between points at: {}, {}", idx, last_point_idx);
            //throw std::runtime_error("WHAT WHAT!");
        }

        last_point_idx = idx;
        idx-=4;
    }

    std::sort(points.begin(), points.end(), [](const auto& a, const auto& b) { return a.angle < b.angle; });

    if (points.size() < 3) {
        return points;
    }

    std::vector<LaserPoint> fixed_points;
    fixed_points.reserve(points.size() + 10);

    float step_size = (points.back().angle - points.front().angle) / (points.size() - 1);
    float max_step = step_size * 1.8;
    fixed_points.push_back(points[0]);
    for (size_t i = 1; i < points.size(); i++) {
        if (points[i].angle - points[i - 1].angle > max_step) {
            debug("Unexpected angle gap between points {} and {}: {}, expected {}", i - 1, i, points[i].angle - points[i - 1].angle, step_size);

            if (fill_gaps) {
                LaserPoint new_point;
                new_point.distance = 100000;
                new_point.angle = points[i - 1].angle + step_size;
                new_point.valid = false;
                fixed_points.push_back(new_point);
            }
        }

        fixed_points.push_back(points[i]);
    }

    return fixed_points;
}

}  // namespace opensw
