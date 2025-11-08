#include "turtlebot3_node/sensors/robit_psd.hpp"

#include <memory>
#include <string>
#include <utility>

using robotis::turtlebot3::sensors::RobitPSD;

RobitPSD::RobitPSD(std::shared_ptr<rclcpp::Node>& nh, const std::string& topic_name) : Sensors(nh) {
  pub_ = nh->create_publisher<std_msgs::msg::UInt16MultiArray>(topic_name, this->qos_);

  RCLCPP_INFO(nh_->get_logger(), "Succeeded to create robit psd publisher");
}

void RobitPSD::publish(const rclcpp::Time& now, std::shared_ptr<DynamixelSDKWrapper>& dxl_sdk_wrapper) {
  (void)now;
  auto msg = std::make_unique<std_msgs::msg::UInt16MultiArray>();

  uint16_t psd_l = dxl_sdk_wrapper->get_data_from_device<uint16_t>(extern_control_table.robit_psd_l.addr, extern_control_table.robit_psd_l.length);
  uint16_t psd_f = dxl_sdk_wrapper->get_data_from_device<uint16_t>(extern_control_table.robit_psd_f.addr, extern_control_table.robit_psd_f.length);
  uint16_t psd_r = dxl_sdk_wrapper->get_data_from_device<uint16_t>(extern_control_table.robit_psd_r.addr, extern_control_table.robit_psd_r.length);

  msg->data.push_back(psd_l);
  msg->data.push_back(psd_f);
  msg->data.push_back(psd_r);

  pub_->publish(std::move(msg));
}
