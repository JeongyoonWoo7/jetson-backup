#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

class PsdSafetyController : public rclcpp::Node
{
public:
  PsdSafetyController()
  : Node("psd_safety_controller"),
    state_("NORMAL")
  {
   
    this->declare_parameter<double>("normal_speed", 0.18);  // SAFE일 때 전진 속도
    this->declare_parameter<double>("slow_speed",   0.08);  // CAUTION일 때 전진 속도

    normal_speed_ = this->get_parameter("normal_speed").as_double();
    slow_speed_   = this->get_parameter("slow_speed").as_double();

    RCLCPP_INFO(
      this->get_logger(),
      "PsdSafetyController started. normal_speed=%.2f, slow_speed=%.2f",
      normal_speed_, slow_speed_
    );

    // /psd/status 구독
    status_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/psd/status",
      rclcpp::QoS(10),
      std::bind(&PsdSafetyController::statusCallback, this, std::placeholders::_1)
    );

    // /cmd_vel 퍼블리셔
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // 10Hz로 속도 명령 퍼블리시
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&PsdSafetyController::timerCallback, this)
    );
  }

private:
  void statusCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    last_status_ = msg->data;

    // 문자열 안에 DANGER / CAUTION 있는지 검사
    if (msg->data.find("DANGER") != std::string::npos) {
      state_ = "STOP";
    } else if (msg->data.find("CAUTION") != std::string::npos) {
      state_ = "SLOW";
    } else {
      state_ = "NORMAL";
    }

    RCLCPP_INFO(
      this->get_logger(),
      "status recv: '%s' -> state: %s",
      last_status_.c_str(), state_.c_str()
    );
  }

  void timerCallback()
  {
    geometry_msgs::msg::Twist cmd;

    if (state_ == "STOP") {
      cmd.linear.x  = 0.0;
      cmd.angular.z = 0.0;   // 완전 정지
    } else if (state_ == "SLOW") {
      cmd.linear.x  = slow_speed_;
      cmd.angular.z = 0.1;   // 필요하면 회피용 회전 속도도 줄 수 있음
    } else { // "NORMAL"
      cmd.linear.x  = normal_speed_;
      cmd.angular.z = 0.0;
    }

    cmd_pub_->publish(cmd);
  }

  // 파라미터
  double normal_speed_;
  double slow_speed_;

  // 상태
  std::string state_;
  std::string last_status_;

  // ROS 통신
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PsdSafetyController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

