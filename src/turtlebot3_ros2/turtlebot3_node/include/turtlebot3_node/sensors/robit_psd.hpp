#ifndef TURTLEBOT3_NODE__SENSORS__ROBIT_PSD_HPP_
#define TURTLEBOT3_NODE__SENSORS__ROBIT_PSD_HPP_

#include <std_msgs/msg/u_int16_multi_array.hpp>

#include <memory>
#include <string>

#include "turtlebot3_node/sensors/sensors.hpp"

namespace robotis
{
namespace turtlebot3
{
namespace sensors
{
class RobitPSD : public Sensors
{
public:
  explicit RobitPSD(
    std::shared_ptr<rclcpp::Node> & nh,
    const std::string & topic_name = "robit_psd");

  void publish(
    const rclcpp::Time & now,
    std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper) override;

private:
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr pub_;
};
}  // namespace sensors
}  // namespace turtlebot3
}  // namespace robotis
#endif  // TURTLEBOT3_NODE__SENSORS__ROBIT_PSD_HPP_
