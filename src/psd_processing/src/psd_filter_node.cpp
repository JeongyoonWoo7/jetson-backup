#include <memory>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

class PsdFilterNode : public rclcpp::Node
{
public:
  PsdFilterNode()
  : Node("psd_filter_node"),
    raw_front_(0), raw_left_(0), raw_right_(0),
    has_data_(false)
  {
    
    this->declare_parameter<int>("danger_raw", 600);   // 이 값보다 크면 DANGER
    this->declare_parameter<int>("caution_raw", 400);  // 이 값보다 크면 CAUTION

    danger_raw_  = this->get_parameter("danger_raw").as_int();
    caution_raw_ = this->get_parameter("caution_raw").as_int();

    RCLCPP_INFO(
      this->get_logger(),
      "PsdFilterNode started. danger_raw=%d, caution_raw=%d",
      danger_raw_, caution_raw_
    );

    // robit_psd: UInt16MultiArray, data = [left, front, right]
    sub_psd_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
      "/robit_psd",
      rclcpp::QoS(10),
      std::bind(&PsdFilterNode::psdCallback, this, std::placeholders::_1)
    );

    status_pub_ = this->create_publisher<std_msgs::msg::String>("/psd/status", 10);

    // 10Hz 타이머: /psd/status 계속 퍼블리시
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&PsdFilterNode::timerCallback, this)
    );
  }

private:
  void psdCallback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 3) {
      RCLCPP_WARN(this->get_logger(), "robit_psd size < 3 (size=%zu)", msg->data.size());
      return;
    }

    // robit_psd.cpp 기준: [left, front, right]
    raw_front_  = msg->data[0];
    raw_left_ = msg->data[1];
    raw_right_ = msg->data[2];
    has_data_  = true;

    RCLCPP_INFO(this->get_logger(),
      "psdCallback raw L/F/R = %u / %u / %u",
      raw_left_, raw_front_, raw_right_);
  }

  std::string zone(int val) const
  {
    
    if (val > danger_raw_) {
      return "DANGER";
    } else if (val > caution_raw_) {
      return "CAUTION";
    } else {
      return "SAFE";
    }
  }

  void timerCallback()
  {
    std_msgs::msg::String status_msg;
    std::ostringstream ss;

    if (!has_data_) {
      ss << "front:UNKNOWN, left:UNKNOWN, right:UNKNOWN";
    } else {
      std::string z_front = zone(raw_front_);
      std::string z_left  = zone(raw_left_);
      std::string z_right = zone(raw_right_);

      ss << "front:" << z_front << "(" << raw_front_ << "), "
         << "left:"  << z_left  << "(" << raw_left_  << "), "
         << "right:" << z_right << "(" << raw_right_ << ")";
    }

    status_msg.data = ss.str();
    status_pub_->publish(status_msg);

    RCLCPP_INFO(this->get_logger(), "status: %s", status_msg.data.c_str());
  }

  int danger_raw_;
  int caution_raw_;

  uint16_t raw_front_, raw_left_, raw_right_;
  bool has_data_;

  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr sub_psd_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PsdFilterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

