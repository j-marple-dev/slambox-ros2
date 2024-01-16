/// @file
/// @author Jongkuk Lim <limjk@jmarple.ai>
/// @copyright 2023 J.Marple
/// @brief SLAMBOX ROS Driver client node

#include <glog/logging.h>
#include <rclcpp/rclcpp.hpp>

#include "applications/driver_client.hpp"

/// @brief Main node for SLAMBOX Driver Server
/// @param argc Number of arguments
/// @param argv arguments
///
/// @return 0
int main(int argc, char **argv) {
  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;
  google::InitGoogleLogging(argv[0]);

  rclcpp::init(argc, argv);
  auto slambox_client_node = std::make_shared<sbox::SLAMBOXDriverClient>();

  rclcpp::spin(slambox_client_node);
  rclcpp::shutdown();
  return 0;
}
