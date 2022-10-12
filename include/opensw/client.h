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

#include <optional>
#include <random>
#include <string>
#include <system_error>

#include <asio.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opensw/protocol.h>
#include <opensw/types.h>

namespace opensw {

/**
 * Primary class used to communicate with device.
 */
class Client {
  public:
    Client();
    ~Client() = default;

    /**
     * Connect to device at host:port over TCP.
     *
     * @param[in] host  The hostname or IP address of the device.
     * @param[in] port  The port to connect to.
     * @param[in] timeout_ms  Connection timeout in milliseconds.
     *
     * @returns True if the connection is successful, false otherwise.
     */
    bool connect(const std::string& host, int port, int timeout_ms);

    /**
     * Check if the device is currently connected.
     *
     * @returns True if the device is connected, false otherwise.
     */
    bool connected() const;

    /**
     * Disconnect from the device.
     */
    void disconnect();

    /**
     * Get the current device batter percentage from device.
     *
     * Only applies to devices with a battery.
     *
     * @returns The battery percentage if the request is successfull, empty otherwise.
     */
    std::optional<int> getBatteryPercentage();

    /**
     * Get map data layer as a bitmap from device.
     *
     * @param[in] x  The min x extent of the map in meters.
     * @param[in] y  The min y extent of the map in meters.
     * @param[in] width  The width of the map in meters.
     * @param[in] heigher  The height of the map in meters.
     * @param[in] kind  The map layer to retrieve.
     *
     * @returns The map data layer if the request is successful, null otherwise.
     */
    Bitmap::Ptr getBitmapData(float x, float y, float width, float height, MapKind kind);

    /**
     * Get the current device board temperature.
     *
     * @returns The board temperature if the request is successfull, empty otherwise.
     */
    std::optional<int> getBoardTemperature();

    /**
     * Get whether or not DC IN is connected from device.
     *
     * Only applies to devices with a battery.
     *
     * @returns Whether or not DC IN is connected if the request is successful, empty otherwise.
     */
    std::optional<int> getDcInConnected();

    /**
     * Get health status information from device.
     *
     * @returns Health status information if the request is successful, empty otherwise.
     */
    std::optional<HealthStatus> getHealthStatus();

    /**
     * Get IMU data from device.
     *
     * Note: currently only the orientation fields appear to be filled in.
     *
     * @returns IMU data if the request is successful, empty otherwise.
     */
    std::optional<ImuData> getImuData();

    /**
     * Get whether or not the battery is charging from device.
     *
     * Only applies to devices with a battery.
     *
     * @returns Whether or not the battery is charging if the request is successful, empty otherwise.
     */
    std::optional<int> getIsCharging();

    /**
     * Get the current extents of a map data layer from the device.
     *
     * @param[in] type  The type of map.
     * @param[in] kind  The map layer to get the extents of.
     *
     * @returns The map layer extents if the request is successful, empty otherwise.
     */
    std::optional<cv::Rect_<float>> getKnownArea(MapType type, MapKind kind);

    /**
     * Get laser scan data from device.
     *
     * @returns Laser scan data if the request is successful, empty otherwise.
     */
    LaserScan::Ptr getLaserScan();

    /**
     * Get location estimate from device.
     *
     * @returns Location estimate if the request is successful, empty otherwise.
     */
    std::optional<Eigen::Vector3d> getLocation();

    /**
     * Get whether or not the device is on the dock from device.
     *
     * Only applies to devices with a dock.
     *
     * @returns Whether or not the device is on the dock if the request is successful, empty otherwise.
     */
    std::optional<int> getOnDock();

    /**
     * Get pose estimate from device.
     *
     * @returns Pose estimate if the request is successful, empty otherwise.
     */
    std::optional<Eigen::Isometry3d> getPose();

    /**
     * Get IMU data from device.
     *
     * Note: currently only the orientation fields appear to be filled in.
     *
     * @returns IMU data if the request is successful, empty otherwise.
     */
    std::optional<ImuData> getRawImuData();

    /**
     * Get the SPD version from the device.
     *
     * @returns The SPD version if the request is successful, empty otherwise.
     */
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
    int timeout_ms_ = 500;
    bool connection_failed_ = false;
};

}  // namespace opensw
