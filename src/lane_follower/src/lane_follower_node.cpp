#include "lane_follower_node.hpp"

LaneFollowerNode::LaneFollowerNode(const std::string &node_name) : rclcpp::Node(node_name)
{
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    subscriber_ = this->create_subscription<vision_msgs::msg::LaneInfo>(
        "/lane_data",
        10,
        std::bind(&LaneFollowerNode::data_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Lane follower ready.");
}

void LaneFollowerNode::data_callback(const vision_msgs::msg::LaneInfo::SharedPtr msg)
{
    float spd = 1, angle = 0;
    if (!msg->left_detected&&msg->right_detected)angle = -100;
    else if (msg->left_detected&&!msg->right_detected)angle = 100;
    else if (msg->left_detected&&msg->right_detected)angle = msg->lane_angle;
    else spd = 0;
    auto output = std::make_unique<geometry_msgs::msg::Twist>();

    output->linear.x = spd;
    output->angular.z = angle * OFFSET; 
    
    publisher_->publish(std::move(output));
}

int main(int argc, char **argv)
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaneFollowerNode>("lane_follower_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}