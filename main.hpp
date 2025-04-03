#pragma once

#include <rs_driver/api/lidar_driver.hpp>
#include <rs_driver/msg/point_cloud_msg.hpp>
#include <iostream>
#include <string>
#include <memory>

/**
 * @brief Callback function for point cloud messages
 * @param msg  The received message
 */
template <typename T_Point>
void pointCloudCallback(const rs::PointCloudMsg<T_Point>& msg) {
  std::cout << "Point Cloud Callback: Received point cloud with " 
            << msg.point_cloud_ptr->size() << " points." << std::endl;
  std::cout << "Frame ID: " << msg.frame_id << std::endl;
  std::cout << "Timestamp: " << msg.timestamp << std::endl;
}

/**
 * @brief Callback function for exception messages
 * @param code The error code
 * @param msg The error message
 */
void exceptionCallback(const int code, const std::string& msg) {
  std::cout << "Error Code: " << code << ", Error Message: " << msg << std::endl;
}

/**
 * @brief LidarClient class for handling Airy LiDAR sensor connection
 */
class LidarClient {
public:
  LidarClient();
  ~LidarClient();

  /**
   * @brief Initialize the LiDAR driver with configuration
   * @return true if initialization was successful, false otherwise
   */
  bool initialize();

  /**
   * @brief Start the LiDAR driver to receive data
   * @return true if start was successful, false otherwise
   */
  bool start();

  /**
   * @brief Stop the LiDAR driver
   */
  void stop();

private:
  // Create a driver instance
  std::shared_ptr<rs::LidarDriver<rs::LidarPointCloudMsg>> driver_ptr_;
  rs::RSDriverParam driver_param_;  // Configuration parameters for the driver
};