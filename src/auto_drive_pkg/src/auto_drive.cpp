#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <algorithm>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "vision_msgs/msg/lane_info.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

class AutoDrivingNode : public rclcpp::Node
{
public:
  AutoDrivingNode()
  : Node("auto_driving_node"),
    has_lane_info_(false),
    last_lane_detected_(false),
    has_yaw_(false),
    has_target_(false),
    has_data_(false),
    mode_(Mode::STOP),  // 처음엔 정지 상태로 시작
    go_state_(GoAroundState::IDLE),
    prev_lane_detected_for_go_(false)
  {
    // 파라미터
    kp_offset_   = this->declare_parameter("kp_offset",   1.0);
    kp_angle_    = this->declare_parameter("kp_angle",    0.8);
    base_speed_  = this->declare_parameter("base_speed",  0.15);
    max_ang_vel_ = this->declare_parameter("max_ang_vel", 1.2);
    loop_hz_     = this->declare_parameter("loop_hz",     20.0);
    debug_print_ = this->declare_parameter("debug_print", true);
    back_speed_  = this->declare_parameter("back_speed",  0.08);

    // PSD
    danger_raw_  = this->declare_parameter<int>("danger_raw", 600);
    caution_raw_ = this->declare_parameter<int>("caution_raw", 400);
    sub_psd_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
      "/robit_psd", 10, std::bind(&AutoDrivingNode::psdCallback, this, _1));

    // IMU
    kp_      = this->declare_parameter<double>("kp", 1.5);
    tol_deg_ = this->declare_parameter<double>("tol_deg", 2.0);
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu", 50, std::bind(&AutoDrivingNode::imuCallback, this, _1));

    // MASTER 명령 구독 (토픽 이름 변경)
    drive_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/drive_topic", 10, std::bind(&AutoDrivingNode::driveCallback, this, _1));

    // LaneInfo
    lane_sub_ = this->create_subscription<vision_msgs::msg::LaneInfo>(
      "/lane_info", 10, std::bind(&AutoDrivingNode::laneCallback, this, _1));

    // cmd_vel 발행
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // 제어 루프
    auto period_ms = static_cast<int>(1000.0 / loop_hz_);
    if (period_ms <= 0) period_ms = 50;
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&AutoDrivingNode::controlLoop, this));
  }

private:
  // ===== 모드 정의 =====
  enum class Mode {
    STOP,
    NORMAL,
    GO_AROUND,
    PARKING
  };

  enum class GoAroundState {
    IDLE,
    WAIT_BLOCK,
    TURN_RIGHT_90,
    SEARCH_LANE_AFTER_RIGHT,
    FORWARD_ALONG_LANE,
    TURN_LEFT_90
  };

  // ========== PSD ==========
  void psdCallback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 3) return;
    raw_front_ = msg->data[0];
    raw_left_  = msg->data[1];
    raw_right_ = msg->data[2];
    has_data_  = true;
  }

  std::string zone(int val) const
  {
    if (val > danger_raw_)       return "DANGER";
    else if (val > caution_raw_) return "CAUTION";
    else                         return "SAFE";
  }

  // ========== IMU ==========
  double quatToYaw(const geometry_msgs::msg::Quaternion & q_msg)
  {
    tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return yaw;
  }

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

  // ========== MASTER 명령 수신 ==========
  void driveCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string & cmd = msg->data;

    if (cmd == "stop") {
      mode_ = Mode::STOP;
      RCLCPP_WARN(this->get_logger(), "[CMD] STOP 명령 수신");
    }
    else if (cmd == "go") {
      mode_ = Mode::NORMAL;
      RCLCPP_INFO(this->get_logger(), "[CMD] GO 명령 수신 → 자율주행 시작");
    }
    else if (cmd == "parking") {
      mode_ = Mode::PARKING;
      RCLCPP_INFO(this->get_logger(), "[CMD] PARKING 명령 수신 (현재 미구현)");
    }
    else if (cmd == "go_around_block") {
      mode_ = Mode::GO_AROUND;
      go_state_ = GoAroundState::WAIT_BLOCK;
      prev_lane_detected_for_go_ = false;
      RCLCPP_INFO(this->get_logger(), "[CMD] GO_AROUND_BLOCK 명령 수신 → 블럭 회피 모드 진입");
    }
    else if (cmd == "turn_left") {
      has_target_ = true;
      target_yaw_ = current_yaw_ + M_PI / 2.0;
      RCLCPP_INFO(this->get_logger(), "[CMD] TURN_LEFT 명령 수신");
    }
    else if (cmd == "turn_right") {
      has_target_ = true;
      target_yaw_ = current_yaw_ - M_PI / 2.0;
      RCLCPP_INFO(this->get_logger(), "[CMD] TURN_RIGHT 명령 수신");
    }
  }

  // ========== Lane ==========
  void laneCallback(const vision_msgs::msg::LaneInfo::SharedPtr msg)
  {
    last_lane_info_ = *msg;
    has_lane_info_  = true;
  }

  // ========== 블럭 회피 상태머신 (기존과 동일) ==========
  void runGoAround(geometry_msgs::msg::Twist & cmd)
  {
    cmd.linear.x = cmd.angular.z = 0.0;
    std::string front_zone = zone(raw_front_);

    // 블럭 감지 → 오른쪽 90도 회전
    if (go_state_ == GoAroundState::WAIT_BLOCK && front_zone == "DANGER") {
      has_target_ = true;
      target_yaw_ = current_yaw_ - M_PI / 2.0;
      go_state_ = GoAroundState::TURN_RIGHT_90;
      RCLCPP_INFO(this->get_logger(), "[GoAround] 블럭 감지 → 오른쪽 90도 회전 시작");
      return;
    }

    // 회전 완료 → 라인 탐색
    if (go_state_ == GoAroundState::TURN_RIGHT_90 && !has_target_) {
      go_state_ = GoAroundState::SEARCH_LANE_AFTER_RIGHT;
      RCLCPP_INFO(this->get_logger(), "[GoAround] 오른쪽 회전 완료 → 라인 탐색");
    }

    // 라인 찾기 / 직진 / 왼쪽 복귀 로직 (간략 버전)
    const auto & v = last_lane_info_;
    bool line = (v.left_detected || v.right_detected);
    if (go_state_ == GoAroundState::SEARCH_LANE_AFTER_RIGHT) {
      if (line) {
        go_state_ = GoAroundState::FORWARD_ALONG_LANE;
        RCLCPP_INFO(this->get_logger(), "[GoAround] 라인 감지 → 직진 시작");
      } else {
        cmd.linear.x = 0.05;
        return;
      }
    }
    if (go_state_ == GoAroundState::FORWARD_ALONG_LANE) {
      if (!line) {
        has_target_ = true;
        target_yaw_ = current_yaw_ + M_PI / 2.0;
        go_state_ = GoAroundState::TURN_LEFT_90;
        RCLCPP_INFO(this->get_logger(), "[GoAround] 라인 유실 → 왼쪽 90도 회전");
      } else {
        cmd.linear.x = base_speed_;
        return;
      }
    }
  }

  // ========== 메인 제어 루프 ==========
  void controlLoop()
  {
    geometry_msgs::msg::Twist cmd;

    // 1️⃣ IMU 회전 제어 (공통)
    if (has_target_) {
      double err = angleDiff(target_yaw_, current_yaw_);
      double tol_rad = tol_deg_ * M_PI / 180.0;
      if (std::fabs(err) < tol_rad) {
        has_target_ = false;
        cmd_pub_->publish(cmd);
        RCLCPP_INFO(this->get_logger(), "[IMU] 회전 완료");
        return;
      }
      double w = kp_ * err;
      w = std::clamp(w, -max_ang_vel_, max_ang_vel_);
      cmd.angular.z = w;
      cmd.linear.x = 0.0;
      cmd_pub_->publish(cmd);
      return;
    }

    // 2️⃣ 모드 분기
    switch (mode_) {
      case Mode::STOP:
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        cmd_pub_->publish(cmd);
        return;

      case Mode::PARKING:
        // 주차 알고리즘은 다른 담당자 구현 예정
        cmd.linear.x = cmd.angular.z = 0.0;
        cmd_pub_->publish(cmd);
        return;

      case Mode::GO_AROUND:
        runGoAround(cmd);
        cmd_pub_->publish(cmd);
        return;

      case Mode::NORMAL:
        runNormal(cmd);
        cmd_pub_->publish(cmd);
        return;
    }
  }

  // ========== 기본 자율주행 ==========
  void runNormal(geometry_msgs::msg::Twist & cmd)
  {
    if (!has_lane_info_) return;
    const auto & v = last_lane_info_;
    bool left = v.left_detected;
    bool right = v.right_detected;
    if (!left && !right) { cmd.linear.x = 0.0; cmd.angular.z = 0.0; return; }

    double offset_err = v.lane_center_offset;
    double angle_err = v.lane_angle;
    double w = kp_offset_ * offset_err + kp_angle_ * angle_err;
    w = std::clamp(w, -max_ang_vel_, max_ang_vel_);

    double speed_scale = std::max(0.2, 1.0 - std::fabs(offset_err));
    cmd.linear.x = base_speed_ * speed_scale;
    cmd.angular.z = w;
  }

  // ===== 멤버 변수 =====
  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr sub_psd_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr drive_sub_;
  rclcpp::Subscription<vision_msgs::msg::LaneInfo>::SharedPtr lane_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  vision_msgs::msg::LaneInfo last_lane_info_;
  bool has_lane_info_;
  bool last_lane_detected_;

  bool has_data_;
  uint16_t raw_front_, raw_left_, raw_right_;
  int danger_raw_, caution_raw_;

  bool has_yaw_, has_target_;
  double current_yaw_, target_yaw_;
  double kp_, tol_deg_;

  double kp_offset_, kp_angle_, base_speed_, max_ang_vel_, loop_hz_, back_speed_;
  bool debug_print_;

  Mode mode_;
  GoAroundState go_state_;
  bool prev_lane_detected_for_go_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AutoDrivingNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
