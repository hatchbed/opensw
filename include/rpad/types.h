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

#include <memory>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>

namespace rpad {

enum class MapType {
    Bitmap8Bit = 0,
    Pointmap,
    ImageFeaturesMap,
    RectangleAreaMap
};

enum class MapKind {
    EXPLORERMAP = 0,
    COSTMAP = 5,
    SWEEPERMAP  = 10,
    UWBMAP = 20,
    SLAMMAP = 30,
    LOCALSLAMMAP = 40,
    DBOWMAP = 50,
    OPTIMALOPERATIONALMAP = 60,
    MARKERMAP = 70,
    RECTANGLEAREAMAP = 80,
    DISCREPANCYRECORDMAP = 90
};

struct Bitmap {
    using Ptr = std::shared_ptr<Bitmap>;
    using ConstPtr = std::shared_ptr<const Bitmap>;

    cv::Rect_<float> rect;
    float resolution;
    MapKind kind;
    cv::Mat data;
};

struct HealthStatus {
    bool has_depth_camera_disconnected;
    bool has_error;
    bool has_fatal;
    bool has_lidar_disconnected;
    bool has_sdp_disconnected;
    bool has_system_emergency_stop;
    bool has_warning;
};

struct ImuData {
    Eigen::Vector3d acceleration;
    Eigen::Vector3d angular_rate;
    Eigen::Vector3d compass;
    Eigen::Quaternion<double> orientation;
};

struct LaserPoint {
    float angle;
    float distance;
    bool valid;
};

struct LaserScan {
    using Ptr = std::shared_ptr<LaserScan>;
    using ConstPtr = std::shared_ptr<const LaserScan>;

    float x;
    float y;
    float yaw;
    std::vector<LaserPoint> points;
};

}  // namespace rpad
