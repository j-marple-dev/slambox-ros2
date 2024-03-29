/// @file
/// @author Jongkuk Lim <limjk@jmarple.ai>
/// @copyright 2023 J.Marple
/// @brief sbox_msg and ROS message converter

#ifndef SLAMBOX_ROS2_INCLUDE_UTILS_ROS_MSG_CONVERTER_HPP_
#define SLAMBOX_ROS2_INCLUDE_UTILS_ROS_MSG_CONVERTER_HPP_

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <sbox/sbox_msgs/Odometry.hpp>
#include <sbox/sbox_msgs/PointCloud2.hpp>

namespace sbox_msgs {
nav_msgs::msg::Odometry to_ros_msg(const Odometry &odom);
sensor_msgs::msg::PointCloud2 to_ros_msg(const PointCloud2 &pointcloud);
}  // namespace sbox_msgs

#endif  // SLAMBOX_ROS2_INCLUDE_UTILS_ROS_MSG_CONVERTER_HPP_
