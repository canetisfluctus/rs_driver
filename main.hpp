#ifndef MAIN_HPP
#define MAIN_HPP

#include <rs_driver/api/lidar_driver.hpp>
#include <rs_driver/msg/point_cloud_msg.hpp>
#include <memory>

class LidarClient {
public:
  LidarClient();
  ~LidarClient();

  bool initialize();
  bool start();
  void stop();

private:
  robosense::lidar::RSDriverParam driver_param_;
  std::shared_ptr<robosense::lidar::LidarDriver<robosense::lidar::PointCloudMsg<robosense::lidar::PointXYZI>>> driver_ptr_;
};

#endif // MAIN_HPP