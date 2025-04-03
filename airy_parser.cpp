#include <rs_driver/api/lidar_driver.hpp>
#include <rs_driver/msg/point_cloud_msg.hpp>
#include <iostream>
#include <string>
#include <vector>

using namespace robosense::lidar;

class LidarPointCloudCallback {
public:
  void operator()(const PointCloudMsg<PointXYZI>& msg) {
    std::cout << "Point cloud received: " << std::endl;
    std::cout << "Frame ID: " << msg.frame_id << std::endl;
    std::cout << "Timestamp: " << msg.timestamp << std::endl;
    std::cout << "Height: " << msg.height << ", Width: " << msg.width << std::endl;
    std::cout << "Total points: " << msg.points.size() << std::endl;

    uint32_t pointsToPrint = static_cast<uint32_t>(std::min(size_t(10), msg.points.size()));
    for (uint32_t i = 0; i < pointsToPrint; i++) {
      const auto& point = msg.points[i];
      std::cout << "Point " << i << ": "
                << "x=" << point.x << ", "
                << "y=" << point.y << ", "
                << "z=" << point.z << ", "
                << "intensity=" << point.intensity << std::endl;
    }
    std::cout << "----------------" << std::endl;
  }
};

int main(int argc, char** argv) {
  LidarDriver<PointXYZI> driver;

  RSDriverParam param;
  param.input_type = InputType::ONLINE_LIDAR;
  param.input_param.msop_port = 6699;
  param.input_param.difop_port = 7788;
  param.lidar_type = LidarType::RSAIRY;

  driver.regPointCloudCallback([](const PointCloudMsg<PointXYZI>& msg) {
      LidarPointCloudCallback()(msg);
  });

  if (!driver.init(param)) {
    std::cerr << "Driver initialization failed!" << std::endl;
    return -1;
  }

  driver.start();

  std::cout << "Press Ctrl+C to exit" << std::endl;
  while (true) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  return 0;
}