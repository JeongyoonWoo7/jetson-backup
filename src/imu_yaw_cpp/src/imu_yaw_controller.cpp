#include <memory>
#include <cmath>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

class ImuYawController : public rclcpp::Node
{
public:
  ImuYawController()
  : Node("imu_yaw_controller"),
    has_yaw_(false),
    has_target_(false),
    current_yaw_(0.0),
    target_yaw_(0.0)
  {
    // 파라미터: 이득, 최대 각속도, 허용 오차
    kp_ = this->declare_parameter<double>("kp", 1.5);             // P 게인
    max_ang_vel_ = this->declare_parameter<double>("max_ang_vel", 1.0); // rad/s
    tol_deg_ = this->declare_parameter<double>("tol_deg", 2.0);   // 몇 도 이내면 도달로 볼건지

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu", 50, std::bind(&ImuYawController::imuCallback, this, _1));

    target_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "/yaw_target", 10, std::bind(&ImuYawController::targetCallback, this, _1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // 주기적으로 제어 루프 실행 (50 Hz)
    control_timer_ = this->create_wall_timer(
      20ms, std::bind(&ImuYawController::controlLoop, this));

    RCLCPP_INFO(this->get_logger(),
      "ImuYawController started. Subscribing /imu, /yaw_target, publishing /cmd_vel");
  }

private:
  // 쿼터니언 -> yaw
  double quatToYaw(const geometry_msgs::msg::Quaternion & q_msg)
  {
    tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return yaw;
  }

  // -pi ~ pi 범위로 정규화된 (target - current) 각도 오차
  double angleDiff(double target, double current)
  {
    double diff = target - current;
    while (diff > M_PI)  diff -= 2.0 * M_PI;
    while (diff < -M_PI) diff += 2.0 * M_PI;
    return diff;
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    current_yaw_ = quatToYaw(msg->orientation);
    has_yaw_ = true;
  }

  // /yaw_target : "지금 위치에서 +data(rad) 만큼 돌아라"
  void targetCallback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    if (!has_yaw_) {
      RCLCPP_WARN(this->get_logger(),
        "No IMU data yet. Ignoring target command.");
      return;
    }

    // 상대 회전각 (rad)를 현재 yaw에 더해서 절대 목표 yaw로 만들기
    double relative = msg->data;
    target_yaw_ = current_yaw_ + relative; 

    // -pi ~ pi 범위로 맞춰두기 (꼭 필수는 아님, 취향)
    while (target_yaw_ > M_PI)  target_yaw_ -= 2.0 * M_PI;
    while (target_yaw_ < -M_PI) target_yaw_ += 2.0 * M_PI;

    has_target_ = true;

    RCLCPP_INFO(this->get_logger(),
      "New target received: relative=%.3f rad (%.1f deg), "
      "current_yaw=%.3f rad (%.1f deg), target_yaw=%.3f rad (%.1f deg)",
      relative, relative * 180.0 / M_PI,
      current_yaw_, current_yaw_ * 180.0 / M_PI,
      target_yaw_, target_yaw_ * 180.0 / M_PI);
  }

  void controlLoop()
  {
    // yaw 데이터/타겟 없으면 아무 것도 안 함
    if (!has_yaw_ || !has_target_) {
      return;
    }

    double err = angleDiff(target_yaw_, current_yaw_);
    double err_deg = err * 180.0 / M_PI;
    double tol_rad = tol_deg_ * M_PI / 180.0;

    geometry_msgs::msg::Twist cmd;

    // 허용 오차 안에 들어오면 정지 + 타겟 해제
    if (std::fabs(err) < tol_rad) {
      cmd.angular.z = 0.0;
      cmd_pub_->publish(cmd);
      has_target_ = false;
      RCLCPP_INFO(this->get_logger(),
        "Target reached. error=%.3f rad (%.1f deg). Stop.", err, err_deg);
      return;
    }

    // P 제어
    double w = kp_ * err;

    // saturate
    if (w > max_ang_vel_)  w = max_ang_vel_;
    if (w < -max_ang_vel_) w = -max_ang_vel_;

    cmd.angular.z = w;
    cmd.linear.x = 0.0;

    cmd_pub_->publish(cmd); 

    RCLCPP_INFO(this->get_logger(),
      "Control: err=%.3f rad (%.1f deg), cmd_w=%.3f rad/s",
      err, err_deg, w);
  }

  // ROS 인터페이스
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // 상태
  bool has_yaw_;
  bool has_target_;
  double current_yaw_;
  double target_yaw_;

  // 파라미터
  double kp_;
  double max_ang_vel_;
  double tol_deg_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImuYawController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
