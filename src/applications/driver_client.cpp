/// @file
/// @author Jongkuk Lim <limjk@jmarple.ai>
/// @copyright 2023 J.Marple
/// @brief SLAMBOX ROS Driver Client

#include "applications/driver_client.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>  // NOLINT
#include <memory>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>

#include <sbox/communication/serial_communication.hpp>
#include <sbox/protocol/acknowledge/ping_protocol.hpp>
#include <sbox/protocol/base_protocol.hpp>
#include <sbox/protocol/command/command_save_pcd_protocol.hpp>
#include <sbox/protocol/request/request_ethernet_communication_config.hpp>
#include <sbox/protocol/request/request_mavlink_communication_config.hpp>
#include <sbox/protocol/request/request_serial_communication_config.hpp>
#include "utils/ros_msg_converter.hpp"
#include "utils/string_utils.hpp"

namespace sbox {

SLAMBOXDriverClient::SLAMBOXDriverClient(const rclcpp::NodeOptions &options)
    : Node("slambox_ros2_client_node", options),
      serial_parser_(4096),
      udp_parser_(131070) {
  // declare parameters
  this->declare_parameter<std::string>("serial_communication.port_name",
                                       "/dev/ttyUSB1");
  this->declare_parameter<int>("serial_communication.baudrate", 921600);

  this->declare_parameter<bool>("serial_communication.enabled", true);
  this->declare_parameter<bool>("ethernet_communication.enabled", false);

  this->declare_parameter<std::string>("ethernet_communication.server_addr",
                                       "");
  this->declare_parameter<int>("ethernet_communication.port", 21580);

  this->declare_parameter<std::string>("publish.odom_topic", "/SLAMBOX/odom");
  this->declare_parameter<std::string>("publish.pointcloud_topic",
                                       "/SLAMBOX/pointcloud");
  this->declare_parameter<std::string>("subscribe.request_topic",
                                       "/SLAMBOX/request");

  // get parameters
  this->get_parameter_or<std::string>("serial_communication.port_name",
                                      serial_port_name_, "/dev/ttyUSB1");
  this->get_parameter_or<int>("serial_communication.baudrate",
                              serial_baud_rate_, 921600);

  this->get_parameter_or<bool>("serial_communication.enabled",
                               is_serial_enabled_, true);
  this->get_parameter_or<bool>("ethernet_communication.enabled",
                               is_ethernet_enabled_, false);

  this->get_parameter_or<std::string>("ethernet_communication.server_addr",
                                      udp_ip_, "");
  this->get_parameter_or<int>("ethernet_communication.port", udp_port_, 21580);

  this->get_parameter_or<std::string>("publish.odom_topic", publish_odom_topic_,
                                      "/SLAMBOX/odom");
  this->get_parameter_or<std::string>("publish.pointcloud_topic",
                                      publish_pointcloud_topic_,
                                      "/SLAMBOX/pointcloud");
  this->get_parameter_or<std::string>(
      "subscribe/request_topic", subscribe_request_topic_, "/SLAMBOX/request");

  // rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(20)).best_effort();

  odom_pub_ =
      this->create_publisher<nav_msgs::msg::Odometry>(publish_odom_topic_, 10);
  pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      publish_pointcloud_topic_, rclcpp::SystemDefaultsQoS());

  request_sub_ = this->create_subscription<std_msgs::msg::String>(
      subscribe_request_topic_, 1,
      std::bind(&SLAMBOXDriverClient::callback_request_, this,
                std::placeholders::_1));

  cur_pcl_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  clock_ = this->get_clock();

  if (is_serial_enabled_) {
    serial_communication_ = std::make_unique<SerialCommunication>(
        serial_port_name_, serial_baud_rate_);

    serial_parser_.add_parsed_message_callback(this);
    serial_communication_->set_callback(&SBoxParser::parse, &serial_parser_);
    serial_communication_->run();
  }

  if (is_ethernet_enabled_) {
    udp_communication_ =
        std::make_unique<UDPCommunication>(false, udp_ip_, udp_port_);
    udp_parser_.add_parsed_message_callback(this);
    udp_communication_->set_callback(&SBoxParser::parse, &udp_parser_);
    udp_communication_->run();
  }

  send_ping_thread_ =
      std::thread(&SLAMBOXDriverClient::thread_send_ping_, this);
}

SLAMBOXDriverClient::~SLAMBOXDriverClient() {
  this->is_send_ping_thread_running_ = false;
  send_ping_thread_.join();

  if (serial_communication_ != nullptr) {
    serial_communication_->stop();
  }

  if (udp_communication_ != nullptr) {
    udp_communication_->stop();
  }
}

void SLAMBOXDriverClient::thread_send_ping_() {
  while (rclcpp::ok() && this->is_send_ping_thread_running_) {
    if (is_serial_enabled_) {
      if (!this->is_server_alive()) {
        serial_communication_->set_baudrate(115200);
        serial_communication_->write(PingProtocol().encapsulate());
        serial_communication_->set_baudrate(this->serial_baud_rate_);
      }
      serial_communication_->write(PingProtocol().encapsulate());

      LOG(INFO) << "baudrate " << serial_communication_->get_baudrate()
                << std::endl;
    }

    if (is_ethernet_enabled_) {
      udp_communication_->write(PingProtocol().encapsulate());
    }

    if (!is_server_alive()) {
      LOG(WARNING) << "Server is not alive" << std::endl;
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

// cppcheck-suppress unusedFunction
void SLAMBOXDriverClient::on_push_odometry(const sbox_msgs::Odometry &odom) {
  nav_msgs::msg::Odometry odom_msg = sbox_msgs::to_ros_msg(odom);
  odom_pub_->publish(odom_msg);
}

// cppcheck-suppress unusedFunction
void SLAMBOXDriverClient::on_push_pointcloud(
    const sbox_msgs::PointCloud2 &pointcloud) {
  sensor_msgs::msg::PointCloud2 pointcloud_msg =
      sbox_msgs::to_ros_msg(pointcloud);

  if (!have_called_) {
    cur_pcl_timestamp_ = pointcloud_msg.header.stamp;
    pcl::fromROSMsg(pointcloud_msg, *this->cur_pcl_);
    this->cur_pcl_msg_ = pointcloud_msg;
    have_called_ = true;
  }
  if (cur_pcl_timestamp_ == pointcloud_msg.header.stamp) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud =
        std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(pointcloud_msg, *cloud);
    *this->cur_pcl_ += *cloud;
  } else {
    cur_pcl_timestamp_ = pointcloud_msg.header.stamp;
    sensor_msgs::msg::PointCloud2 concat_pcl_msg;
    pcl::toROSMsg(*this->cur_pcl_, concat_pcl_msg);
    concat_pcl_msg.header = cur_pcl_msg_.header;
    pointcloud_pub_->publish(concat_pcl_msg);

    // cur_pcl_.reset();
    pcl::fromROSMsg(pointcloud_msg, *this->cur_pcl_);
    this->cur_pcl_msg_ = pointcloud_msg;
  }
}

// cppcheck-suppress unusedFunction
void SLAMBOXDriverClient::on_response_mavlink_communication_config(
    bool enabled, uint32_t baudrate) {
  LOG(INFO) << "[mavlink] " << (enabled ? "Enabled" : "Disabled")
            << ", baudrate: " << baudrate << std::endl;
}

// cppcheck-suppress unusedFunction
void SLAMBOXDriverClient::on_response_serial_communication_config(
    bool enabled, uint32_t baudrate) {
  LOG(INFO) << "[serial] " << (enabled ? "Enabled" : "Disabled")
            << ", baudrate: " << baudrate << std::endl;
}

// cppcheck-suppress unusedFunction
void SLAMBOXDriverClient::on_response_ethernet_communication_config(
    bool enabled, uint32_t port) {
  LOG(INFO) << "[ethernet] " << (enabled ? "Enabled" : "Disabled")
            << ", port: " << port << std::endl;
}

// cppcheck-suppress unusedFunction
void SLAMBOXDriverClient::on_acknowledge(std::array<uint8_t, 2> requested_mode,
                                         uint8_t status) {
  LOG(INFO) << "[Acknowledge] Requested mode: " << requested_mode[0] << " "
            << requested_mode[1] << ",  Status: " << status;

  if (requested_mode == sbox::protocol::kModeAckPing) {
    last_ping_time_ = std::chrono::system_clock::now();
  }
}

bool SLAMBOXDriverClient::is_server_alive() {
  std::chrono::duration<double> dt =
      std::chrono::system_clock::now() - last_ping_time_;
  // LOG(INFO) << "Server live check: " << dt.count();
  if (dt.count() > 3.0) {
    return false;
  }
  return true;
}

void SLAMBOXDriverClient::callback_request_(const std_msgs::msg::String &msg) {
  std::vector<std::string> split_msg = string_utils::split(msg.data, ',');
  std::vector<uint8_t> data;
  if (split_msg.front() == "GET") {
    if (split_msg[1] == "DATA_STATUS") {
      std::vector<uint8_t> protocol1 =
          RequestMavlinkCommunicationConfigProtocol().encapsulate();
      std::vector<uint8_t> protocol2 =
          RequestSerialCommunicationConfigProtocol().encapsulate();
      std::vector<uint8_t> protocol3 =
          RequestEthernetCommunicationConfigProtocol().encapsulate();
      data.insert(data.end(), protocol1.begin(), protocol1.end());
      data.insert(data.end(), protocol2.begin(), protocol2.end());
      data.insert(data.end(), protocol3.begin(), protocol3.end());
    }

  } else if (split_msg.front() == "CMD") {
    if (split_msg[1] == "SAVE_PCD" && split_msg.size() >= 4) {
      bool save, reset;
      save = (split_msg[2] == "1");
      reset = (split_msg[3] == "1");
      CommandSavePCDProtocol command_save_pcd_protocol(save, reset);
      data = command_save_pcd_protocol.encapsulate();
    }
  }
  if (data.size() < 8) {
    LOG(WARNING) << "Unknown request: " << msg.data;
    return;
  }

  if (is_serial_enabled_) {
    serial_communication_->write(data);
  }

  if (is_ethernet_enabled_) {
    udp_communication_->write(data);
  }
}
}  // namespace sbox
