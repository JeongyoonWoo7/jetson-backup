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
    has_data_(false)
  {
    // 파라미터
    kp_offset_   = this->declare_parameter("kp_offset",   1.0);   // 중앙 오프셋 보정 이득
    kp_angle_    = this->declare_parameter("kp_angle",    0.8);   // 차선 각도 보정 이득
    base_speed_  = this->declare_parameter("base_speed",  0.15);  // 기본 선속도 (m/s)
    max_ang_vel_ = this->declare_parameter("max_ang_vel", 1.2);   // 최대 각속도 (rad/s)
    loop_hz_     = this->declare_parameter("loop_hz",     20.0);  // 제어 주기(Hz)
    debug_print_ = this->declare_parameter("debug_print", true);  // 디버깅 로그 on/off

    // 후진 속도 (양수로 받고 내부에서 -를 곱해서 사용)
    back_speed_  = this->declare_parameter("back_speed",  0.08);  // m/s

    // PSD 파라미터
    danger_raw_  = this->declare_parameter<int>("danger_raw", 600);
    caution_raw_ = this->declare_parameter<int>("caution_raw", 400);

    sub_psd_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
      "/robit_psd", 10, std::bind(&AutoDrivingNode::psdCallback, this, _1));

    // IMU Yaw 제어
    kp_      = this->declare_parameter<double>("kp", 1.5);
    tol_deg_ = this->declare_parameter<double>("tol_deg", 2.0);

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu", 50, std::bind(&AutoDrivingNode::imuCallback, this, _1));

    target_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/master_cmd", 10, std::bind(&AutoDrivingNode::targetCallback, this, _1));

    // LaneInfo 구독
    lane_sub_ = this->create_subscription<vision_msgs::msg::LaneInfo>(
      "/lane_info", 10, std::bind(&AutoDrivingNode::laneCallback, this, _1));

    // cmd_vel 발행
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // 제어 루프 타이머
    auto period_ms = static_cast<int>(1000.0 / loop_hz_);
    if (period_ms <= 0) period_ms = 50;
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&AutoDrivingNode::controlLoop, this));
  }

private:
  // ========== PSD ==========
  void psdCallback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 3) {
      RCLCPP_WARN(this->get_logger(), "robit_psd size < 3 (size=%zu)", msg->data.size());
      return;
    }

    // [front, left, right] 라고 가정 (네 코드에 맞춰 조정)
    raw_front_ = msg->data[0];
    raw_left_  = msg->data[1];
    raw_right_ = msg->data[2];
    has_data_  = true;
  }

  std::string zone(int val) const
  {
    if (val > danger_raw_)      return "DANGER";
    else if (val > caution_raw_) return "CAUTION";
    else                        return "SAFE";
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

  void targetCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (!has_yaw_) {
      RCLCPP_WARN(this->get_logger(), "No IMU data yet. Ignoring target command.");
      return;
    }

    has_target_ = true;
    if (msg->data == "turn_left") {
      target_yaw_ = current_yaw_ + M_PI / 2.0;
    } else if (msg->data == "turn_right") {
      target_yaw_ = current_yaw_ - M_PI / 2.0;
    }
  }

  // ========== Lane ==========
  void laneCallback(const vision_msgs::msg::LaneInfo::SharedPtr msg)
  {
    last_lane_info_ = *msg;
    has_lane_info_ = true;

    bool lane_detected = (msg->left_detected || msg->right_detected);

    if (lane_detected != last_lane_detected_) {
      if (lane_detected) {
        RCLCPP_INFO(this->get_logger(),
          "[Lane] 차선 재인식! left=%d, right=%d, offset=%.3f, angle=%.3f",
          msg->left_detected, msg->right_detected,
          msg->lane_center_offset, msg->lane_angle);
      } else {
        RCLCPP_WARN(this->get_logger(),
          "[Lane] 차선 유실! left=%d, right=%d",
          msg->left_detected, msg->right_detected);
      }
      last_lane_detected_ = lane_detected;
    }

    if (debug_print_) {
      RCLCPP_DEBUG_THROTTLE(
        this->get_logger(), *this->get_clock(), 200,
        "[Lane raw] left=%d, right=%d, offset=%.3f, angle=%.3f",
        msg->left_detected, msg->right_detected,
        msg->lane_center_offset, msg->lane_angle);
    }
  }

  // ========== 메인 제어 루프 ==========
  void controlLoop()
  {
    geometry_msgs::msg::Twist cmd;

    // 1) IMU 타겟 회전 모드
    if (has_target_) {
      double err = angleDiff(target_yaw_, current_yaw_);
      double tol_rad = tol_deg_ * M_PI / 180.0;

      if (std::fabs(err) < tol_rad) {
        has_target_ = false;
        cmd_pub_->publish(cmd);
        RCLCPP_INFO(this->get_logger(), "Target reached. Stop.");
        return;
      }

      double w = kp_ * err;
      if (w > max_ang_vel_)  w = max_ang_vel_;
      if (w < -max_ang_vel_) w = -max_ang_vel_;

      cmd.angular.z = w;
      cmd.linear.x = 0.0;
      cmd_pub_->publish(cmd);
      return;
    }

    // 2) PSD 앞쪽 장애물 → 정지
    if (zone(raw_front_) == "DANGER") {
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      cmd_pub_->publish(cmd);
      RCLCPP_WARN(this->get_logger(), "앞에 장애물 감지! 자동 주행 정지!");
      return;
    }

    // 3) 아직 LaneInfo 없음 → 정지
    if (!has_lane_info_) {
      cmd_pub_->publish(cmd);
      if (debug_print_) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "[Control] 아직 LaneInfo를 받은 적이 없습니다. 정지 상태 유지.");
      }
      return;
    }

    // 4) Lane 기반 제어
    const auto & v = last_lane_info_;
    bool left  = v.left_detected;
    bool right = v.right_detected;

    double offset_err = static_cast<double>(v.lane_center_offset); // [-1,1] 가정
    double angle_err  = static_cast<double>(v.lane_angle);

    // NaN 방어
    if (!std::isfinite(offset_err)) offset_err = 0.0;
    if (!std::isfinite(angle_err))  angle_err  = 0.0;

    // 마지막으로 어느 쪽 차선을 봤는지 기억
    static bool last_seen_left  = false;
    static bool last_seen_right = false;

    if (left && !right) {
      last_seen_left  = true;
      last_seen_right = false;
    } else if (!left && right) {
      last_seen_left  = false;
      last_seen_right = true;
    } else if (left && right) {
      last_seen_left  = true;
      last_seen_right = true;
    }

    // 4-1) 한쪽만 보이는 경우 → 유실된 방향으로 회전하며 찾기
    if (left && !right) {
      // 오른쪽 라인 유실 → 오른쪽으로 회전
      cmd.linear.x = 0.0;
      cmd.angular.z = -0.6;   // 오른쪽 회전
      cmd_pub_->publish(cmd);
      if (debug_print_) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 500,
          "[Control] 오른쪽 라인 유실 → 오른쪽 회전 탐색");
      }
      return;
    }

    if (!left && right) {
      // 왼쪽 라인 유실 → 왼쪽으로 회전
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.6;    // 왼쪽 회전
      cmd_pub_->publish(cmd);
      if (debug_print_) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 500,
          "[Control] 왼쪽 라인 유실 → 왼쪽 회전 탐색");
      }
      return;
    }

    // 4-2) 둘 다 유실 → 후진
    if (!left && !right) {
      cmd.linear.x = -back_speed_;  // 천천히 후진
      cmd.angular.z = 0.0;
      cmd_pub_->publish(cmd);
      if (debug_print_) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 500,
          "[Control] 양쪽 라인 유실 → 후진 (v=%.2f)", cmd.linear.x);
      }
      return;
    }

    // 4-3) 양쪽 라인 모두 잘 보이는 정상 상태 → PID 주행
    double w = kp_offset_ * offset_err + kp_angle_ * angle_err;
    if (w > max_ang_vel_)  w = max_ang_vel_;
    if (w < -max_ang_vel_) w = -max_ang_vel_;

    double speed_scale = std::max(0.2, 1.0 - std::fabs(offset_err)); // 최소 0.2
    double v_lin = base_speed_ * speed_scale;

    cmd.linear.x = v_lin;
    cmd.angular.z = w;
    cmd_pub_->publish(cmd);

    if (debug_print_) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 500,
        "[Control] offset=%.3f, angle=%.3f -> v=%.2f, w=%.2f (scale=%.2f)",
        offset_err, angle_err, v_lin, w, speed_scale);
    }
  }

  // ========== 멤버 변수들 ==========
  // PSD
  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr sub_psd_;
  int danger_raw_;
  int caution_raw_;
  uint16_t raw_front_, raw_left_, raw_right_;
  bool has_data_;

  // IMU
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr target_sub_;
  bool has_yaw_;
  bool has_target_;
  double current_yaw_;
  double target_yaw_;
  double kp_;
  double tol_deg_;

  // Lane
  rclcpp::Subscription<vision_msgs::msg::LaneInfo>::SharedPtr lane_sub_;
  vision_msgs::msg::LaneInfo last_lane_info_;
  bool has_lane_info_;
  bool last_lane_detected_;

  // cmd_vel + 루프
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // 제어 파라미터
  double kp_offset_;
  double kp_angle_;
  double base_speed_;
  double max_ang_vel_;
  double loop_hz_;
  double back_speed_;
  bool   debug_print_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AutoDrivingNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
