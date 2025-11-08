#include "turtlebot3_node/sensors/robit_button.hpp"

#include <memory>
#include <string>
#include <utility>

using robotis::turtlebot3::sensors::RobitButton;

RobitButton::RobitButton(std::shared_ptr<rclcpp::Node>& nh, const std::string& topic_name) : Sensors(nh) {
  pub_ = nh->create_publisher<std_msgs::msg::Int8MultiArray>(topic_name, this->qos_);

  RCLCPP_INFO(nh_->get_logger(), "Succeeded to create robit button publisher");
}

void RobitButton::publish(const rclcpp::Time& now, std::shared_ptr<DynamixelSDKWrapper>& dxl_sdk_wrapper) {
  (void)now;
  auto msg = std::make_unique<std_msgs::msg::Int8MultiArray>();

  uint8_t button_data = dxl_sdk_wrapper->get_data_from_device<uint8_t>(extern_control_table.robit_button.addr, extern_control_table.robit_button.length);

  for (int i = 0; i < 8; i++) {
    msg->data.push_back(static_cast<bool>(button_data & (1 << i)));
  }

  pub_->publish(std::move(msg));
}
