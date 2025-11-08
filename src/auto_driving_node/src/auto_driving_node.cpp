#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "vision_msgs/msg/lane_info.hpp"

using namespace std::chrono_literals;

class AutoDrivingNode : public rclcpp::Node
{
public:
  AutoDrivingNode()
  : Node("auto_driving_node"),
    has_lane_info_(false),
    last_lane_detected_(false)
  {
    // 파라미터
    kp_offset_   = this->declare_parameter("kp_offset",   1.0);   // 중앙 오프셋 보정 이득
    kp_angle_    = this->declare_parameter("kp_angle",    0.8);   // 차선 각도 보정 이득
    base_speed_  = this->declare_parameter("base_speed",  0.15);  // 기본 선속도 (m/s)
    max_ang_vel_ = this->declare_parameter("max_ang_vel", 1.2);   // 최대 각속도 (rad/s)
    loop_hz_     = this->declare_parameter("loop_hz",     20.0);  // 제어 주기(Hz) - 디버깅용으로 조정 가능
    debug_print_ = this->declare_parameter("debug_print", true);  // 디버깅 로그 on/off

    auto period_ms = static_cast<int>(1000.0 / loop_hz_);
    if (period_ms <= 0) period_ms = 50;  // 방어 코드

    RCLCPP_INFO(this->get_logger(),
                "AutoDrivingNode initialized. "
                "kp_offset=%.3f, kp_angle=%.3f, base_speed=%.3f, max_ang_vel=%.3f, loop_hz=%.1f",
                kp_offset_, kp_angle_, base_speed_, max_ang_vel_, loop_hz_);

    // LaneInfo 구독
    lane_sub_ = this->create_subscription<vision_msgs::msg::LaneInfo>(
      "/lane_info", 10,
      std::bind(&AutoDrivingNode::laneCallback, this, std::placeholders::_1));

    // cmd_vel 발행
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // 주기 제어 루프
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&AutoDrivingNode::controlLoop, this));
  }

private:
  void laneCallback(const vision_msgs::msg::LaneInfo::SharedPtr msg)
  {
    last_lane_info_ = *msg;
    has_lane_info_ = true;

    bool lane_detected = (msg->left_detected || msg->right_detected);

    // lane 인식 상태가 바뀔 때마다 INFO로 알려주기 (디버그할 때 굉장히 도움됨)
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

    // high-rate 디버그는 DEBUG 레벨 + THROTTLE
    if (debug_print_) {
      RCLCPP_DEBUG_THROTTLE(
        this->get_logger(), *this->get_clock(), 200,  // 200ms에 한 번
        "[Lane raw] left=%d, right=%d, offset=%.3f, angle=%.3f",
        msg->left_detected, msg->right_detected,
        msg->lane_center_offset, msg->lane_angle);
    }
  }

  void controlLoop()
  {
    geometry_msgs::msg::Twist cmd;

    if (!has_lane_info_) {
      // 아직 비전 데이터 없음 → 정지
      if (debug_print_) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "[Control] 아직 LaneInfo를 받은 적이 없습니다. 정지 상태 유지.");
      }
      cmd_pub_->publish(cmd);
      return;
    }

    const auto & v = last_lane_info_;

    // 왼쪽/오른쪽 차선 둘 다 안 보이면 → 안전하게 정지
    if (!(v.left_detected || v.right_detected)) {
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      cmd_pub_->publish(cmd);

      if (debug_print_) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 500,
          "[Control] 차선 미검출 → 정지 (left=%d, right=%d)",
          v.left_detected, v.right_detected);
      }
      return;
    }

    // lane_center_offset: 화면 중앙 기준 [-1, 1]
    // lane_angle: 차선 기울기 (rad)
    double offset_err = static_cast<double>(v.lane_center_offset);
    double angle_err  = static_cast<double>(v.lane_angle);

    // NaN / Inf 방어
    if (!std::isfinite(offset_err)) {
      RCLCPP_WARN(this->get_logger(), "[Control] offset_err is not finite! -> 0으로 대체");
      offset_err = 0.0;
    }
    if (!std::isfinite(angle_err)) {
      RCLCPP_WARN(this->get_logger(), "[Control] angle_err is not finite! -> 0으로 대체");
      angle_err = 0.0;
    }

    // P 제어 (중앙 오프셋 + 각도 둘 다 이용)
    double w = kp_offset_ * offset_err + kp_angle_ * angle_err;

    // 각속도 제한
    if (w > max_ang_vel_)  w = max_ang_vel_;
    if (w < -max_ang_vel_) w = -max_ang_vel_;

    // 중앙에서 멀어질수록 속도 줄이기
    double speed_scale = std::max(0.0, 1.0 - 0.5 * std::fabs(offset_err));
    double v_lin = base_speed_ * speed_scale;

    cmd.linear.x = v_lin;
    cmd.angular.z = w;
    cmd_pub_->publish(cmd);

    // 디버그 출력 (0.5초에 한 번)
    if (debug_print_) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 500,
        "[Control] offset=%.3f, angle=%.3f -> v=%.2f, w=%.2f (scale=%.2f)",
        offset_err, angle_err, v_lin, w, speed_scale);
    }
  }

  // 멤버 변수들
  rclcpp::Subscription<vision_msgs::msg::LaneInfo>::SharedPtr lane_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  vision_msgs::msg::LaneInfo last_lane_info_;
  bool has_lane_info_;
  bool last_lane_detected_;   // 차선 인식 상태 변화 감지용

  double kp_offset_;
  double kp_angle_;
  double base_speed_;
  double max_ang_vel_;
  double loop_hz_;            // 제어 주기 (Hz)
  bool   debug_print_;        // 디버그 로그 on/off
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // 로그 레벨을 DEBUG로 두면 위 DEBUG 로그까지 다 보임
  rclcpp::NodeOptions options;
  auto node = std::make_shared<AutoDrivingNode>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
