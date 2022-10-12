# opensw
Open source client SDK for communicating with SlamTec Slamware devices such as
the [M2M1 mapper](https://www.slamtec.com/en/Lidar/Mapper) over ethernet.

An official SDK is [available](https://www.slamtec.com/en/Support#rplidar-mapper),
but appears to be closed source.

Note: For the standalone SlamTec lidar devices, an existing open source driver
is already available: [rplidar_ros](https://github.com/Slamtec/rplidar_ros), but
this uses a different protocol than the mapper devices.

Currently, the interface is only partially implemented to expose the lidar scan
and IMU data as well as some of the mapping data. More of the interface provided
by the official SDK can be easily added, as needed, since the protocol is
straightforward JSON over TCP.  These additional interfaces appear to support
their robotic platforms, which use the same protocol.

## Build Dependencies

- [asio](https://think-async.com/Asio/) for TCP communication
- [asyncapi_gencpp](https://github.com/hatchbed/asyncapi_gencpp) for json 
    protocol serialization/deserialization code generation from an
    [asyncapi](https://www.asyncapi.com/) style spec file.
- [eigen](https://eigen.tuxfamily.org/) for some datatypes
- [gtest](https://github.com/google/googletest) for unit testing
- [opencv](https://opencv.org/) for some datatypes
- [spdlog](https://github.com/gabime/spdlog) for formatted logging

## Supported Commands

- `GetImuInRobotCoordinate`: Gets IMU sensor data
- `GetKnownArea`: Gets the current bounds of a map
- `GetLaserScan`: Gets laser scan data
- `GetLocation`: Gets the current location estimate
- `GetMapData`: Gets raw map data
- `GetPose`: Gets the current pose estimate
- `GetRobotHealth`: Gets device health status data
- `GetSpdVersion`: Gets version information
- `GetSystemResource`: Gets system information

## Example Usage:
```
#include <opensw/client.h>

int main(int argc, char **argv) {

  opensw::Client client;
  if (client.connect("192.168.11.11", 1445, 500)) {
    auto scan = client.getLaserScan();
  }
}

```

## ROS Support

This library is agnostic to ROS, but is packaged to work in a ROS1 or ROS2
workspace.

See [opensw_ros](https://github.com/hatchbed/opensw_ros) for a simple ROS laser scan
and IMU driver built by wrapping this SDK.