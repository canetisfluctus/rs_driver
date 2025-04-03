#include "main.hpp"
#include <thread>
#include <chrono>

LidarClient::LidarClient() {
  driver_ptr_ = std::make_shared<rs::LidarDriver<rs::LidarPointCloudMsg>>();
}

LidarClient::~LidarClient() {
  stop();
}

bool LidarClient::initialize() {
  // For Airy LiDAR, set the correct parameters
  driver_param_.lidar_type = "RSAIRY";  // Choose your LiDAR type
  driver_param_.msop_port = 6699;      // MSOP port, default is 6699
  driver_param_.difop_port = 7788;     // DIFOP port, default is 7788
  //driver_param_.decoder_param.use_lidar_clock = true;  // Use LiDAR clock as timestamp
  
  // Set the point cloud callback function
  driver_ptr_->regPointCloudCallback(pointCloudCallback<rs::LidarPointCloudMsg::PointT>);
  
  // Set the exception callback function
  driver_ptr_->regExceptionCallback(exceptionCallback);
  
  // Initialize the driver with the parameters
  if (!driver_ptr_->init(driver_param_)) {
    std::cerr << "Failed to initialize LiDAR driver." << std::endl;
    return false;
  }
  
  std::cout << "LiDAR driver initialized successfully." << std::endl;
  return true;
}

bool LidarClient::start() {
  // Start the driver to receive LiDAR data
  if (!driver_ptr_->start()) {
    std::cerr << "Failed to start LiDAR driver." << std::endl;
    return false;
  }
  
  std::cout << "LiDAR driver started successfully. Receiving data..." << std::endl;
  return true;
}

void LidarClient::stop() {
  if (driver_ptr_) {
    driver_ptr_->stop();
    std::cout << "LiDAR driver stopped." << std::endl;
  }
}

int main(int argc, char** argv) {
  std::cout << "Robosense Airy LiDAR Connection Test" << std::endl;
  
  // Create a LiDAR client instance
  LidarClient lidar_client;
  
  // Initialize the LiDAR driver
  if (!lidar_client.initialize()) {
    std::cerr << "Failed to initialize LiDAR client." << std::endl;
    return -1;
  }
  
  // Start the LiDAR driver
  if (!lidar_client.start()) {
    std::cerr << "Failed to start LiDAR client." << std::endl;
    return -1;
  }
  
  std::cout << "LiDAR connection established. Press Ctrl+C to exit." << std::endl;
  
  // Keep the program running to receive LiDAR data
  while (true) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  
  return 0;
}