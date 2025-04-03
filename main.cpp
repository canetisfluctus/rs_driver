#include "main.hpp"
#include <iostream>
#include <thread>
#include <chrono>

LidarClient::LidarClient() {
  driver_ptr_ = std::make_shared<robosense::lidar::LidarDriver<robosense::lidar::PointCloudMsg<robosense::lidar::PointXYZI>>>();
}

LidarClient::~LidarClient() {
  stop();
}

bool LidarClient::initialize() {
  driver_param_.lidar_type = robosense::lidar::LidarType::RSAIRY;
  driver_param_.input_param.msop_port = 6699;
  driver_param_.input_param.difop_port = 7788;

  driver_ptr_->regPointCloudCallback([](const robosense::lidar::PointCloudMsg<robosense::lidar::PointXYZI>& msg) {
    std::cout << "Received point cloud with " << msg.points.size() << " points." << std::endl;
  });

  driver_ptr_->regExceptionCallback([](const robosense::lidar::Error& err) {
    std::cerr << "LiDAR exception: " << err.toString() << std::endl;
  });

  if (!driver_ptr_->init(driver_param_)) {
    std::cerr << "Failed to initialize LiDAR driver." << std::endl;
    return false;
  }

  std::cout << "LiDAR driver initialized successfully." << std::endl;
  return true;
}

bool LidarClient::start() {
  if (!driver_ptr_->start()) {
    std::cerr << "Failed to start LiDAR driver." << std::endl;
    return false;
  }

  std::cout << "LiDAR driver started successfully." << std::endl;
  return true;
}

void LidarClient::stop() {
  if (driver_ptr_) {
    driver_ptr_->stop();
    std::cout << "LiDAR driver stopped." << std::endl;
  }
}

int main(int argc, char** argv) {
  LidarClient client;

  if (!client.initialize()) {
    return -1;
  }

  if (!client.start()) {
    return -1;
  }

  std::cout << "LiDAR is running. Press Ctrl+C to exit." << std::endl;

  while (true) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  return 0;
}