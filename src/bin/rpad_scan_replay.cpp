#include <fstream>
#include <iostream>

#include <asio.hpp>
#include <rpad/logger.h>
#include <rpad/msg/GetLaserScanResponse.h>
#include <rpad/msg/SimpleRequest.h>
#include <rpad/protocol.h>

using asio::ip::tcp;
using namespace spdlog;

int main(int argc, char* argv[]) {

    if (argc != 3) {
      std::cerr << "Usage: rpad_scan_replay <scan_file> <port>\n";
      return 1;
    }

    std::ifstream data_file(argv[1]);
    std::string line;
    while (std::getline(data_file, line) && (line.empty() || line[0] == '#')) {
      // skip blank or comment lines
    }
    info("read {} bytes of scan data from {}", line.size(), argv[1]);

    // TODO validate the data is base64

    int port = std::stoi(argv[2]);
    info("waiting for connection on port {} ...", port);
    asio::io_context io_context;
    tcp::acceptor a(io_context, tcp::endpoint(tcp::v4(), port));
    tcp::socket sock(io_context);
    a.accept(sock);

    info("waiting for request ...");
    asio::streambuf buffer;
    asio::read_until(sock, buffer, rpad::DELIM);
    std::string req {
        asio::buffers_begin(buffer.data()),
        asio::buffers_end(buffer.data()) - rpad::DELIM.size()
    };

    auto request = rpad::msg::SimpleRequest::fromJson(req);
    if (!request) {
        error("failed to parse request");
        return {};
    }

    if (request->command != "getlaserscan") {
        error("unsupported request type: {}", request->command);
    }

    rpad::msg::GetLaserScanResponse response;
    response.request_id = request->request_id;
    response.command = request->command;
    response.result.code = 0;
    response.result.laser_points = line;
    response.result.timestamp = 100;
    response.result.x = 0;
    response.result.y = 0;
    response.result.yaw = 0;

    auto msg = response.dump() + rpad::DELIM;
    asio::write(sock, asio::buffer(&msg[0], msg.size()));

    info("sent response");
    return 0;
}
