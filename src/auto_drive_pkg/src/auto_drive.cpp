#include <chrono>
#include <cmath>
#include <string>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "custom_interfaces/msg/packet_protocol.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std::chrono_literals;

class AutoDrivingNode : public rclcpp::Node
{
public:
  AutoDrivingNode()
  : Node("auto_driving_node"),
    mode_(Mode::STOP),
    go_state_(GoAroundState::IDLE),
    has_packet_(false),
    has_psd_data_(false),
    has_yaw_(false),
    has_target_(false),
    start_flag_(0)   // start 신호 초기값 0 → 정지 상태
  {
    // ===== 파라미터 =====
    base_speed_  = this->declare_parameter("base_speed", 0.3);
    back_speed_  = this->declare_parameter("back_speed", 0.3);
    kp_offset_   = this->declare_parameter("kp_offset", 0.3);
    kp_angle_    = this->declare_parameter("kp_angle", 0.65);
    kp_yaw_      = this->declare_parameter("kp_yaw", 1.2);
    max_ang_vel_ = this->declare_parameter("max_ang_vel", 0.8);
    yaw_tolerance_deg_ = this->declare_parameter("yaw_tol_deg", 2.0);
    danger_raw_  = this->declare_parameter("danger_raw", 600);
    caution_raw_ = this->declare_parameter("caution_raw", 400);

    // ===== 구독 =====
    packet_sub_ = this->create_subscription<custom_interfaces::msg::PacketProtocol>(
      "/packet_protocol", 10, std::bind(&AutoDrivingNode::packetCallback, this, std::placeholders::_1));

    psd_sub_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
      "/robit_psd", 10, std::bind(&AutoDrivingNode::psdCallback, this, std::placeholders::_1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu", 50, std::bind(&AutoDrivingNode::imuCallback, this, std::placeholders::_1));

    drive_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/drive_topic", 10, std::bind(&AutoDrivingNode::driveCallback, this, std::placeholders::_1));

    // ===== 발행 =====
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // ===== 타이머 루프 =====
    timer_ = this->create_wall_timer(50ms, std::bind(&AutoDrivingNode::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "✅ AutoDrivingNode 시작 (PacketProtocol 기반)");
  }

private:
  // ========== 모드 Enum ==========
  enum class Mode { STOP, NORMAL, GO_AROUND, PARKING };
  enum class GoAroundState { IDLE, WAIT_BLOCK, TURN_RIGHT_90, SEARCH_WHITE, FOLLOW_WHITE, TURN_LEFT_90, SEARCH_YELLOW };

  // ========== 콜백 ==========
  void packetCallback(const custom_interfaces::msg::PacketProtocol::SharedPtr msg)
  {
    last_packet_ = *msg;
    has_packet_ = true;

    // start 신호 받아오기 (필요 시 PacketProtocol 내 필드명에 맞게 수정)
    start_flag_ = msg->start;   // 예시: start 필드를 start로 사용한다고 가정
  }

  void psdCallback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 3) return;
    raw_front_ = msg->data[0];
    raw_left_  = msg->data[1];
    raw_right_ = msg->data[2];
    has_psd_data_ = true;
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    current_yaw_ = yaw;
    has_yaw_ = true;
  }

  void driveCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string & cmd = msg->data;

    if (cmd == "stop") {
      mode_ = Mode::STOP;
      RCLCPP_WARN(this->get_logger(), "[CMD] STOP → 정지");
    }
    else if (cmd == "go") {
      mode_ = Mode::NORMAL;
      go_state_ = GoAroundState::IDLE;
      RCLCPP_INFO(this->get_logger(), "[CMD] GO → 일반 주행 시작");
    }
    else if (cmd == "go_around_block") {
      mode_ = Mode::GO_AROUND;
      go_state_ = GoAroundState::WAIT_BLOCK;
      RCLCPP_INFO(this->get_logger(), "[CMD] GO_AROUND_BLOCK → 블럭 회피 모드 진입");
    }
    else if (cmd == "parking") {
      mode_ = Mode::PARKING;
      RCLCPP_INFO(this->get_logger(), "[CMD] PARKING 모드 진입 (현재 비활성)");
    }
    else if (cmd == "turn_left" && has_yaw_) {
      has_target_ = true;
      target_yaw_ = normalizeAngle(current_yaw_ + M_PI / 2.0);
    }
    else if (cmd == "turn_right" && has_yaw_) {
      has_target_ = true;
      target_yaw_ = normalizeAngle(current_yaw_ - M_PI / 2.0);
    }
  }

  // ========== 유틸 ==========
  static double normalizeAngle(double a)
  {
    while (a > M_PI) a -= 2*M_PI;
    while (a < -M_PI) a += 2*M_PI;
    return a;
  }

  static double angleDiff(double t, double c)
  {
    double d = t - c;
    while (d > M_PI) d -= 2*M_PI;
    while (d < -M_PI) d += 2*M_PI;
    return d;
  }

  std::string zone(uint16_t raw)
  {
    if (raw > danger_raw_) return "DANGER";
    else if (raw > caution_raw_) return "CAUTION";
    return "SAFE";
  }

  // ========== 모드별 ==========
void runNormal(geometry_msgs::msg::Twist & cmd)
{
  // 유효 데이터 확인
  if (!has_packet_) return;

  const auto & v = last_packet_;

  // 라인 미검출 시 정지 또는 후진
  if (!v.left_detected_color && !v.right_detected_color) {
    cmd.linear.x = -0.1;
    cmd.angular.z = 0.0;
    return;
  }

  // 기본 오프셋 / 각도 오차
  double offset_err = static_cast<double>(v.lane_center_offset_color);
  double angle_err  = static_cast<double>(v.lane_angle_color);

  // 단순 P 제어
  double w = kp_offset_ * offset_err + kp_angle_ * angle_err;
  w = std::clamp(w, -max_ang_vel_, max_ang_vel_);

  // 속도는 기본 속도 유지
  cmd.linear.x  = base_speed_;
  cmd.angular.z = w;
}





  void runGoAround(geometry_msgs::msg::Twist & cmd)
  {
    cmd.linear.x = cmd.angular.z = 0.0;
    bool danger_front = has_psd_data_ && (zone(raw_front_) == "DANGER");

    switch (go_state_)
    {
      case GoAroundState::WAIT_BLOCK:
        if (danger_front) {
          has_target_ = true;
          target_yaw_ = normalizeAngle(current_yaw_ - M_PI/2.0);
          go_state_ = GoAroundState::TURN_RIGHT_90;
        } else runNormal(cmd);
        break;
      case GoAroundState::TURN_RIGHT_90:
        if (!has_target_) go_state_ = GoAroundState::SEARCH_WHITE;
        break;
      case GoAroundState::SEARCH_WHITE:
        cmd.linear.x = 0.05;
        break;
      case GoAroundState::FOLLOW_WHITE:
        if (danger_front) {
          has_target_ = true;
          target_yaw_ = normalizeAngle(current_yaw_ + M_PI/2.0);
          go_state_ = GoAroundState::TURN_LEFT_90;
        } else runNormal(cmd);
        break;
      case GoAroundState::TURN_LEFT_90:
        if (!has_target_) go_state_ = GoAroundState::SEARCH_YELLOW;
        break;
      case GoAroundState::SEARCH_YELLOW:
        cmd.linear.x = 0.05;
        break;
      default:
        break;
    }
  }

  bool runYawControl(geometry_msgs::msg::Twist & cmd)
  {
    if (!(has_target_ && has_yaw_)) return false;
    double err = angleDiff(target_yaw_, current_yaw_);
    double tol = yaw_tolerance_deg_ * M_PI / 180.0;

    if (std::fabs(err) < tol) {
      has_target_ = false;
      RCLCPP_INFO(this->get_logger(), "[IMU] 회전 완료");
      return true;
    }

    cmd.angular.z = std::clamp(kp_yaw_ * err, -max_ang_vel_, max_ang_vel_);
    cmd.linear.x = 0.0;
    cmd_pub_->publish(cmd);
    return true;
  }

  void controlLoop()
  {
    geometry_msgs::msg::Twist cmd;

    if (start_flag_ == 0) {
      cmd.linear.x = cmd.angular.z = 0.0;
      cmd_pub_->publish(cmd);
      return;
    }

    if (runYawControl(cmd)) return;

    switch (mode_)
    {
      case Mode::STOP: cmd.linear.x = cmd.angular.z = 0.0; break;
      case Mode::NORMAL: runNormal(cmd); break;
      case Mode::GO_AROUND: runGoAround(cmd); break;
      case Mode::PARKING: cmd.linear.x = cmd.angular.z = 0.0; break;
    }

    cmd_pub_->publish(cmd);
  }

  // ========== 멤버 ==========
  rclcpp::Subscription<custom_interfaces::msg::PacketProtocol>::SharedPtr packet_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr psd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr drive_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  custom_interfaces::msg::PacketProtocol last_packet_;
  bool has_packet_, has_psd_data_, has_yaw_, has_target_;
  int start_flag_;

  double current_yaw_, target_yaw_;
  double base_speed_, back_speed_;
  double kp_offset_, kp_angle_, kp_yaw_;
  double max_ang_vel_, yaw_tolerance_deg_;
  int danger_raw_, caution_raw_;
  uint16_t raw_front_, raw_left_, raw_right_;

  Mode mode_;
  GoAroundState go_state_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutoDrivingNode>());
  rclcpp::shutdown();
  return 0;
}
