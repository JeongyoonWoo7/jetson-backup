
#ifndef TURTLEBOT3_NODE__DEVICES__ROBIT_LED_HPP_
#define TURTLEBOT3_NODE__DEVICES__ROBIT_LED_HPP_

#include "std_msgs/msg/u_int8.hpp"

#include "turtlebot3_node/devices/devices.hpp"

namespace robotis {
namespace turtlebot3 {
namespace devices {

class RobitLED {
private:
  /* data */
public:
  RobitLED(std::shared_ptr<rclcpp::Node> &nh,
           std::shared_ptr<DynamixelSDKWrapper> &dxl_sdk_wrapper,
           const std::string &topic_name = "robit_led");
  ~RobitLED();

  void callback(const std_msgs::msg::UInt8::SharedPtr msg) const;

protected:
  std::shared_ptr<rclcpp::Node> nh_;
  std::shared_ptr<DynamixelSDKWrapper> dxl_sdk_wrapper_;
  rclcpp::QoS qos_ = rclcpp::QoS(rclcpp::ServicesQoS());

  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_;
};

} // namespace devices
} // namespace turtlebot3
} // namespace robotis

#endif