#include "turtlebot3_node/devices/robit_led.hpp"

namespace robotis {
namespace turtlebot3 {
namespace devices {

RobitLED::RobitLED(std::shared_ptr<rclcpp::Node> &nh,
                   std::shared_ptr<DynamixelSDKWrapper> &dxl_sdk_wrapper,
                   const std::string &topic_name)
    : nh_(nh), dxl_sdk_wrapper_(dxl_sdk_wrapper) {
  RCLCPP_INFO(nh_->get_logger(), "Succeeded to create robit led subscriber");
  sub_ = nh_->create_subscription<std_msgs::msg::UInt8>(
      topic_name, 10,
      std::bind(&RobitLED::callback, this, std::placeholders::_1));
}
RobitLED::~RobitLED() {}

void RobitLED::callback(const std_msgs::msg::UInt8::SharedPtr msg) const {
  std::string res;
  dxl_sdk_wrapper_->set_data_to_device(
      extern_control_table.robit_mode_led.addr,
      extern_control_table.robit_mode_led.length, &msg->data, &res);
}
} // namespace devices
} // namespace turtlebot3
} // namespace robotis