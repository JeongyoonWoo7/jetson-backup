#include "rclcpp/rclcpp.hpp"
#include <string>
#include <memory>
#include <functional>
#include <vector>
#include <cmath>

#include "geometry_msgs/msg/twist.hpp" 
#include "vision_msgs/msg/lane_info.hpp"

#define SPD 1.0 
#define OFFSET 1.0 

class LaneFollowerNode : public rclcpp::Node
{
public:
    LaneFollowerNode(const std::string &node_name);

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<vision_msgs::msg::LaneInfo>::SharedPtr subscriber_;
    void data_callback(const vision_msgs::msg::LaneInfo::SharedPtr msg);
};