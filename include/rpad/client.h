#pragma once

#include <optional>
#include <random>
#include <string>
#include <system_error>


#include <asio.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <rpad/protocol.h>
#include <rpad/types.h>

namespace rpad {

class Client {
  public:
    Client();
    ~Client() = default;

    bool connect(const std::string& host, int port, int timeout_ms);
    bool connected() const;
    void disconnect();

    std::optional<int> getBatteryPercentage();
    Bitmap::Ptr getBitmapData(float x, float y, float width, float height, MapKind kind);
    std::optional<int> getBoardTemperature();
    std::optional<int> getDcInConnected();
    std::optional<int> getIsCharging();
    std::optional<cv::Rect_<float>> getKnownArea(MapType type, MapKind kind);
    LaserScan::Ptr getLaserScan();
    std::optional<Eigen::Vector3d> getLocation();
    std::optional<int> getOnDock();
    std::optional<Eigen::Isometry3d> getPose();
    std::optional<std::string> getSdpVersion();

  private:
    void handleConnect(const std::error_code& err);
    void checkDeadline(const std::error_code& err);
    std::optional<std::string> sendAndReceive(const std::string& msg);

    const size_t max_buffer_size_ = 10000000;
    asio::io_context io_context_;
    asio::io_context::work work_;
    asio::deadline_timer deadline_;
    asio::ip::tcp::socket socket_;
    asio::streambuf stream_buffer_;
    std::mt19937 rng_;
    std::uniform_int_distribution<std::mt19937::result_type> id_dist_;
    int timeout_ms_;
    bool connection_failed_ = false;
};

}  // namespace rpad
