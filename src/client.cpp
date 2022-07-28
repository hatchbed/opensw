#include <rpad/client.h>

#include <chrono>
#include <iostream>

#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>

#include <rpad/base64.h>
#include <rpad/logger.h>
#include <rpad/msg/messages.h>

using namespace spdlog;

namespace rpad {

Client::Client() :
    socket_(io_context_),
    work_(io_context_),
    deadline_(io_context_),
    stream_buffer_(max_buffer_size_)
{
    std::random_device dev;
    rng_ = std::mt19937(dev());
    id_dist_ = std::uniform_int_distribution<std::mt19937::result_type>(1, 99999999);
}

bool Client::connect(const std::string& host, int port, int timeout_ms) {
    timeout_ms_ = timeout_ms;

    if (socket_.is_open()) {
        return true;
    }

    connection_failed_ = false;
    try {
        asio::ip::tcp::resolver resolver(io_context_);
        asio::ip::tcp::resolver::results_type endpoints = resolver.resolve(host,  std::to_string(port));

        info("Connecting to [{}:{}] ...", host, port);

        asio::async_connect(socket_, endpoints, boost::bind(&Client::handleConnect, this, asio::placeholders::error));

        deadline_.expires_from_now(boost::posix_time::milliseconds(timeout_ms_));
        deadline_.async_wait([&](const std::error_code& err){ checkDeadline(err); });
        do io_context_.run_one(); while (!connection_failed_ && !socket_.is_open());
    }
    catch (const std::exception& e) {
        error("Exception: {}", e.what());
        return false;
    }

    return socket_.is_open();
}

void Client::handleConnect(const std::error_code& err) {
    if (!err) {
        info("Connected.");
    }
    else {
        connection_failed_ = true;
    }
}

bool Client::connected() const {
    return socket_.is_open();
}

void Client::disconnect() {
    socket_.close();
}

std::optional<std::string> Client::getSdpVersion() {
    msg::SimpleRequest req;
    req.request_id = id_dist_(rng_);
    req.command = CMD_GET_SPD_VERSION;
    auto resp = sendAndReceive(req.dump());
    if (!resp) {
        return {};
    }
    auto response = msg::GetSdpVersionResponse::fromJson(*resp);
    if (!response) {
        error("Failed to parse response");
        return {};
    }

    return response->result.sdp_version;
}

std::optional<int> Client::getBatteryPercentage() {
    msg::SimpleRequest req;
    req.request_id = id_dist_(rng_);
    req.command = CMD_GET_SYSTEM_RESOURCE;
    auto resp = sendAndReceive(req.dump());
    if (!resp) {
        return {};
    }
    auto response = msg::GetSystemResourceResponse::fromJson(*resp);
    if (!response) {
        error("Failed to parse response");
        return {};
    }

    return response->result.battery_percentage;
}

Bitmap::Ptr Client::getBitmapData(float x, float y, float width, float height, MapKind kind) {
    msg::GetMapDataRequest req;
    req.request_id = id_dist_(rng_);
    req.command = CMD_GET_MAP_DATA;
    req.args.area.x = x;
    req.args.area.y = y;
    req.args.area.width = width;
    req.args.area.height = height;
    req.args.partially = false;
    req.args.type = static_cast<std::underlying_type_t<MapType>>(MapType::Bitmap8Bit);
    req.args.kind = static_cast<std::underlying_type_t<MapKind>>(kind);
    auto resp = sendAndReceive(req.dump());
    if (!resp) {
        return {};
    }
    auto response = msg::GetMapDataResponse::fromJson(*resp);
    if (!response) {
        error("Failed to parse response");
        return {};
    }

    if (response->result.dimension_x * response->result.dimension_y != response->result.size) {
        error("Dimensions don't match size: {} * {} != {}",
            response->result.dimension_x, response->result.dimension_y, response->result.size);
        return {};
    }

    auto data = inflate(response->result.map_data);
    if (data.size() != response->result.size) {
        error("Inflated data doesn't match expected size: {} != {}", data.size(), response->result.size);
        return {};
    }

    auto bitmap = std::make_shared<Bitmap>();
    bitmap->rect.x = response->result.real_x;
    bitmap->rect.y = response->result.real_y;
    bitmap->rect.width = width;
    bitmap->rect.height = height;
    bitmap->resolution = response->result.resolution;
    bitmap->kind = kind;
    bitmap->data = cv::Mat(response->result.dimension_x, response->result.dimension_y, CV_8U, data.data()).clone();

    return bitmap;
}

std::optional<int> Client::getBoardTemperature() {
    msg::SimpleRequest req;
    req.request_id = id_dist_(rng_);
    req.command = CMD_GET_SYSTEM_RESOURCE;
    auto resp = sendAndReceive(req.dump());
    if (!resp) {
        return {};
    }
    auto response = msg::GetSystemResourceResponse::fromJson(*resp);
    if (!response) {
        error("Failed to parse response");
        return {};
    }

    return response->result.board_temperature10;
}

std::optional<int> Client::getDcInConnected() {
    msg::SimpleRequest req;
    req.request_id = id_dist_(rng_);
    req.command = CMD_GET_SYSTEM_RESOURCE;
    auto resp = sendAndReceive(req.dump());
    if (!resp) {
      return {};
    }
    auto response = msg::GetSystemResourceResponse::fromJson(*resp);
    if (!response) {
        error("Failed to parse response");
        return {};
    }

    return response->result.dcin_connected;
}

std::optional<HealthStatus> Client::getHealthStatus() {
    msg::SimpleRequest req;
    req.request_id = id_dist_(rng_);
    req.command = CMD_GET_ROBOT_HEALTH;
    auto resp = sendAndReceive(req.dump());
    if (!resp) {
        return {};
    }
    auto response = msg::GetRobotHealthResponse::fromJson(*resp);
    if (!response) {
        error("Failed to parse response");
        return {};
    }

    HealthStatus status;
    status.has_depth_camera_disconnected = response->result.has_depth_camera_disconnected;
    status.has_error = response->result.has_error;
    status.has_fatal = response->result.has_fatal;
    status.has_lidar_disconnected = response->result.has_lidar_disconnected;
    status.has_sdp_disconnected = response->result.has_sdp_disconnected;
    status.has_system_emergency_stop = response->result.has_system_emergency_stop;
    status.has_warning = response->result.has_warning;

    return status;
}

std::optional<ImuData> Client::getImuData() {
    msg::SimpleRequest req;
    req.request_id = id_dist_(rng_);
    req.command = CMD_GET_IMU_IN_ROBOT_COORDINATE;
    auto resp = sendAndReceive(req.dump());
    if (!resp) {
        return {};
    }
    auto response = msg::GetImuResponse::fromJson(*resp);
    if (!response) {
        error("Failed to parse response");
        return {};
    }

    ImuData imu_data;
    imu_data.acceleration = { response->result.acc.x, response->result.acc.y, response->result.acc.z };
    imu_data.angular_rate = { response->result.gyro.x, response->result.gyro.y, response->result.gyro.z };
    imu_data.compass = { response->result.compass.x, response->result.compass.y, response->result.compass.z };
    imu_data.orientation = { response->result.quaternion.w, response->result.quaternion.x, response->result.quaternion.y, response->result.quaternion.z };

    return imu_data;
}

std::optional<int> Client::getIsCharging() {
    msg::SimpleRequest req;
    req.request_id = id_dist_(rng_);
    req.command = CMD_GET_SYSTEM_RESOURCE;
    auto resp = sendAndReceive(req.dump());
    if (!resp) {
        return {};
    }
    auto response = msg::GetSystemResourceResponse::fromJson(*resp);
    if (!response) {
        error("Failed to parse response");
        return {};
    }

    return response->result.is_charging;
}

std::optional<cv::Rect_<float>> Client::getKnownArea(MapType type, MapKind kind) {
    msg::GetKnownAreaRequest req;
    req.request_id = id_dist_(rng_);
    req.command = CMD_GET_KNOWN_AREA;
    req.args.type = static_cast<std::underlying_type_t<MapType>>(type);
    req.args.kind = static_cast<std::underlying_type_t<MapKind>>(kind);
    auto resp = sendAndReceive(req.dump());
    if (!resp) {
        return {};
    }
    auto response = msg::GetKnownAreaResponse::fromJson(*resp);
    if (!response) {
        error("Failed to parse response");
        return {};
    }

    return cv::Rect_<float>(
        response->result.min_x,
        response->result.min_y,
        response->result.max_x - response->result.min_x,
        response->result.max_y - response->result.min_y);
}

LaserScan::Ptr Client::getLaserScan() {
    msg::SimpleRequest req;
    req.request_id = id_dist_(rng_);
    req.command = CMD_GET_LASER_SCAN;
    auto resp = sendAndReceive(req.dump());
    if (!resp) {
        return {};
    }

    auto response = msg::GetLaserScanResponse::fromJson(*resp);
    if (!response) {
        error("Failed to parse response");
        return {};
    }

    auto scan = std::make_shared<LaserScan>();
    scan->x = response->result.x;
    scan->y = response->result.y;
    scan->yaw = response->result.yaw;
    scan->points = parseLaserPoints(response->result.laser_points, true);

    return scan;
}

std::optional<Eigen::Vector3d> Client::getLocation() {
    msg::SimpleRequest req;
    req.request_id = id_dist_(rng_);
    req.command = CMD_GET_LOCATION;
    auto resp = sendAndReceive(req.dump());
    if (!resp) {
        return {};
    }
    auto response = msg::GetLocationResponse::fromJson(*resp);
    if (!response) {
        error("Failed to parse response");
        return {};
    }

    return Eigen::Vector3d{response->result.x, response->result.y, 0.0};
}

std::optional<int> Client::getOnDock() {
    msg::SimpleRequest req;
    req.request_id = id_dist_(rng_);
    req.command = CMD_GET_SYSTEM_RESOURCE;
    auto resp = sendAndReceive(req.dump());
    if (!resp) {
        return {};
    }
    auto response = msg::GetSystemResourceResponse::fromJson(*resp);
    if (!response) {
        error("Failed to parse response");
        return {};
    }

    return response->result.on_dock;
}

std::optional<Eigen::Isometry3d> Client::getPose() {
    msg::SimpleRequest req;
    req.request_id = id_dist_(rng_);
    req.command = CMD_GET_POSE;
    auto resp = sendAndReceive(req.dump());
    if (!resp) {
        return {};
    }
    auto response = msg::GetPoseResponse::fromJson(*resp);
    if (!response) {
        error("Failed to parse response");
        return {};
    }

    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation().x() = response->result.x;
    pose.translation().y() = response->result.y;
    Eigen::AngleAxisd roll(response->result.roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch(response->result.pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw(response->result.yaw, Eigen::Vector3d::UnitZ());
    pose.linear() = (yaw * pitch * roll).toRotationMatrix();

    return pose;
}

std::optional<ImuData> Client::getRawImuData() {
    msg::SimpleRequest req;
    req.request_id = id_dist_(rng_);
    req.command = CMD_GET_IMU_IN_ROBOT_COORDINATE;
    auto resp = sendAndReceive(req.dump());
    if (!resp) {
        return {};
    }
    auto response = msg::GetImuResponse::fromJson(*resp);
    if (!response) {
        error("Failed to parse response");
        return {};
    }

    ImuData imu_data;
    imu_data.acceleration = { response->result.raw_acc.x, response->result.raw_acc.y, response->result.raw_acc.z };
    imu_data.angular_rate = { response->result.raw_gyro.x, response->result.raw_gyro.y, response->result.raw_gyro.z };
    imu_data.compass = { response->result.raw_compass.x, response->result.raw_compass.y, response->result.raw_compass.z };
    imu_data.orientation = { response->result.quaternion.w, response->result.quaternion.x, response->result.quaternion.y, response->result.quaternion.z };

    return imu_data;
}

std::optional<std::string> Client::sendAndReceive(const std::string& msg) {
    if (!socket_.is_open()) {
        return {};
    }

    trace("Sending request: [{}]", msg);
    std::string request = msg + DELIM;
    asio::write(socket_, asio::buffer(&request[0], request.size()));

    // make sure the response buffer is empty
    stream_buffer_.consume(stream_buffer_.size());

    std::string resp;
    asio::error_code result = asio::error::would_block;
    asio::async_read_until(socket_, stream_buffer_, DELIM, [&](asio::error_code err, size_t bytes_transferred){
        result = err;
        if (err) {
            error("Error receiving message: {}", err.message());
        }
        else {
            resp = {
                asio::buffers_begin(stream_buffer_.data()),
                asio::buffers_end(stream_buffer_.data()) - DELIM.size()};
            trace("Response: [{}]", resp);
        }
    });

    deadline_.expires_from_now(boost::posix_time::milliseconds(timeout_ms_));
    deadline_.async_wait([&](const std::error_code& err){ checkDeadline(err); });
    do io_context_.run_one(); while (result == asio::error::would_block);

    if (resp.empty()) {
        return {};
    }

    return resp;
}

void Client::checkDeadline(const std::error_code& err) {
    if (deadline_.expires_at() <= asio::deadline_timer::traits_type::now()) {
        error("Socket timed out");
        disconnect();
        deadline_.expires_at(boost::posix_time::pos_infin);
    }
}

}  // namespace rpad



