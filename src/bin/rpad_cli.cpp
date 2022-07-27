#include <iostream>

#include <rpad/client.h>
#include <rpad/logger.h>

using namespace spdlog;

int main(int argc, char* argv[]) {

    if (argc != 3) {
      std::cerr << "Usage: rpad_cli <host> <port>\n";
      return 1;
    }

    rpad::Client client;
    if (!client.connect(argv[1], std::stoi(argv[2]), 500)) {
      error("Failed to connect to device.");
      return 1;
    }

    auto sdp_version = client.getSdpVersion();
    info("SDP Version: {}", sdp_version ? *sdp_version : "Unknown");

    auto battery_percentage = client.getBatteryPercentage();
    if (battery_percentage) {
      std::cout << "Battery Percentage: " << *battery_percentage << "\n";
    }
    else {
      std::cout << "Battery Percentage: Unknown\n";
    }

    auto board_temperature = client.getBoardTemperature();
    if (board_temperature) {
      std::cout << "Board Temperature: " << *board_temperature << "\n";
    }
    else {
      std::cout << "Board Temperature: Unknown\n";
    }

    auto dcin_connected = client.getDcInConnected();
    if (dcin_connected) {
      std::cout << "DCIN Connected: " << *dcin_connected << "\n";
    }
    else {
      std::cout << "DCIN Connected: Unknown\n";
    }

    auto is_charging = client.getIsCharging();
    if (is_charging) {
      std::cout << "Is Charging: " << *is_charging << "\n";
    }
    else {
      std::cout << "Is Charging: Unknown\n";
    }

    auto on_dock = client.getOnDock();
    if (on_dock) {
      std::cout << "On Dock: " << *on_dock << "\n";
    }
    else {
      std::cout << "On Dock: Unknown\n";
    }

    auto explorer_map = client.getBitmapData(-1, -1, 2, 2, rpad::MapKind::EXPLORERMAP);
    if (explorer_map) {
      std::cout << "Explorer Map: \n";
      std::cout << "  x: " << explorer_map->rect.x << "\n";
      std::cout << "  y: " << explorer_map->rect.y << "\n";
      std::cout << "  width: " << explorer_map->rect.width << "\n";
      std::cout << "  height: " << explorer_map->rect.height << "\n";
      std::cout << "  resolution: " << explorer_map->resolution << "\n";
      std::cout << "  rows: " << explorer_map->data.rows << "\n";
      std::cout << "  cols: " << explorer_map->data.rows << "\n";
    }

    return 0;
    auto known_explorer_area = client.getKnownArea(rpad::MapType::Bitmap8Bit, rpad::MapKind::EXPLORERMAP);
    if (known_explorer_area) {
      std::cout << "Known Area (Explorer Map):\n";
      std::cout << "  x: " << known_explorer_area->x << "\n";
      std::cout << "  y: " << known_explorer_area->y << "\n";
      std::cout << "  width: " << known_explorer_area->width << "\n";
      std::cout << "  height: " << known_explorer_area->height << "\n";
    }

    for (size_t i = 0; i < 10000; i++) {
      auto laser_scan = client.getLaserScan();
      if (laser_scan) {

      }
    }
}
