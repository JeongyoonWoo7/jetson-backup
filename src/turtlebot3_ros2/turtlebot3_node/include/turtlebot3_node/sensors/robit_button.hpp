#ifndef TURTLEBOT3_NODE__SENSORS__ROBIT_BUTTON_HPP_
#define TURTLEBOT3_NODE__SENSORS__ROBIT_BUTTON_HPP_

#include <std_msgs/msg/int8_multi_array.hpp>

#include <memory>
#include <string>

#include "turtlebot3_node/sensors/sensors.hpp"

namespace robotis
{
namespace turtlebot3
{
namespace sensors
{
class RobitButton : public Sensors
{
public:
  explicit RobitButton(
    std::shared_ptr<rclcpp::Node> & nh,
    const std::string & topic_name = "robit_button");

  void publish(
    const rclcpp::Time & now,
    std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper) override;

private:
  rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr pub_;
};
}  // namespace sensors
}  // namespace turtlebot3
}  // namespace robotis
#endif  // TURTLEBOT3_NODE__SENSORS__ROBIT_BUTTON_HPP_
