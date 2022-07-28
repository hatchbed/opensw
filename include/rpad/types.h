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
