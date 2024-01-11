/// @file
/// @author Jongkuk Lim <limjk@jmarple.ai>
/// @copyright 2023 J.Marple
/// @brief SLAMBOX ROS Driver Client

#ifndef SLAMBOX_ROS_INCLUDE_APPLICATIONS_DRIVER_CLIENT_HPP_
#define SLAMBOX_ROS_INCLUDE_APPLICATIONS_DRIVER_CLIENT_HPP_

#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <memory>
#include <string>
#include <vector>

#include <sbox/communication/sbox_parser.hpp>
#include <sbox/communication/serial_communication.hpp>
#include <sbox/communication/udp_communication.hpp>


namespace sbox {

/// @brief SLAMBOX Driver Client
class SLAMBOXDriverClient : public ParsedMessageInterface, public rclcpp::Node {
public:
  /// @brief Constructor
  /// @param nh ROS NodeHandle
  /// @param pnh ROS Private NodeHandle
  explicit SLAMBOXDriverClient(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  /// @brief Destructor
  ~SLAMBOXDriverClient();

  /// Check if response of ping has been received from the server
  /// @return true if response of ping has been received from the server within
  /// 3 seconds
  bool is_server_alive();

private:
  /// @brief Callback function for odometry message
  /// @param odom Odometry message
  ///
  /// This function is called when odometry message is received.
  void on_push_odometry(const sbox_msgs::Odometry &odom) override;

  /// @brief Callback function for pointcloud message
  /// @param pointcloud Pointcloud message
  /// @details This function is called when pointcloud message is received.
  void on_push_pointcloud(const sbox_msgs::PointCloud2 &pointcloud) override;

  /// Receieve mavlink communication config
  /// @param enabled mavlink communication enabled flag
  /// @param baudrate mavlink baudrate
  void on_response_mavlink_communication_config(bool enabled,
                                                uint32_t baudrate) override;

  /// Receieve serial communication config
  /// @param enabled serial communication enabled flag
  /// @param baudrate serial baudrate
  void on_response_serial_communication_config(bool enabled,
                                               uint32_t baudrate) override;

  /// Receieve ethernet communication config
  /// @param enabled ethernet communication enabled flag
  /// @param port ethernet port number
  void on_response_ethernet_communication_config(bool enabled,
                                                 uint32_t port) override;

  /// @brief Acknowledge callback
  /// @param requested_mode requested mode
  /// @param status status
  /// @details This function is called when acknowledge is parsed.
  void on_acknowledge(std::array<uint8_t, 2> requested_mode,
                      uint8_t status) override;

  /// @brief Callback function for request message
  /// @param msg request message from ROS to SLAMBOX Driver Client
  void callback_request_(const std_msgs::msg::String &msg);

  /// @brief Callback function for command save pcd
  /// @param msg Command message from ROS to SLAMBOX Driver Client
  void callback_save_pcd_command_(const std_msgs::msg::String &msg);

  /// @brief Thread function for sending ping to server
  /// This function sends a ping message to server every 1 second.
  void thread_send_ping_();

  /// @brief Mavlink communication enabled flag
  bool is_mavlink_enabled_ = false;

  /// @brief Serial communication enabled flag
  bool is_serial_enabled_ = true;

  /// @brief Ethernet communication enabled flag
  bool is_ethernet_enabled_ = false;

  /// @brief Serial port name
  std::string serial_port_name_;

  /// @brief Serial baud rate
  int serial_baud_rate_;

  /// @brief UDP port number
  int udp_port_;

  /// @brief UDP IP address
  std::string udp_ip_;

  /// @brief Parser for serial communication
  sbox::SBoxParser serial_parser_;

  /// @brief Parser for UDP communication
  sbox::SBoxParser udp_parser_;

  /// @brief Serial communication
  std::unique_ptr<SerialCommunication> serial_communication_ = nullptr;

  /// @brief UDP communication
  std::unique_ptr<UDPCommunication> udp_communication_ = nullptr;

  /// @brief Topic name of Odometry message
  std::string publish_odom_topic_;

  /// @brief Topic name of Pointcloud message
  std::string publish_pointcloud_topic_;

  /// @brief Topic name of request message
  std::string subscribe_request_topic_;

  /// @brief topic name of save pcd command
  std::string subscribe_save_pcd_topic_;

  /// @brief Odometry publisher (nav_msgs::Odometry)
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  /// @brief Pointcloud publisher (sensor_msgs::PointCloud2)
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;

  /// @brief Request from ROS subscriber (std_msgs::String)
  /// @details This subscriber is used to receive request from ROS.
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr request_sub_;

  /// @brief Thread for sending ping to server
  std::thread send_ping_thread_;

  /// @brief Flag for running send ping thread
  bool is_send_ping_thread_running_ = true;

  /// @brief Last ping time. This is used to check if server is alive.
  std::chrono::system_clock::time_point last_ping_time_;

  /// @brief ROS2 clock
  rclcpp::Clock::SharedPtr clock_;
};

} // namespace sbox

#endif // SLAMBOX_ROS_INCLUDE_APPLICATIONS_DRIVER_CLIENT_HPP_
