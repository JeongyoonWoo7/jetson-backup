#include <rclcpp/rclcpp.hpp>
#include "custom_interfaces/msg/packet_protocol.hpp"
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>

#include <vector>
#include <chrono>
#include <thread>
using namespace std::chrono_literals;


class master : public rclcpp::Node //서브스크라이버
{
    rclcpp::Subscription<custom_interfaces::msg::PacketProtocol>::SharedPtr sub; //  인클루드한 것으로 타입 지정
    //rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_int;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_psd;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub; //  인클루드한 것으로 타입 지정
    rclcpp::TimerBase::SharedPtr timer_;
    int start; //출발 여부  
    int red_flag; //빨간선 여부
    std::vector<int> red_co; //빨간선 좌표
    int yellow_flag; // 노란선 여부
    std::vector<int> yellow_co; //노란선 좌표
    int white_flag; //흰색 여부
    std::vector<int> white_co; // 흰색선 좌표
    int traffic_flag; //신호등 여부
    int traffic_color; // 신호등 색깔
    int sign_flag; //표지판 여부
    std::vector<int> sign_co; //표지판 좌표
    int what_sign; //표지판 방향
    int gate; //차단기 
    int block; //벽돌 여부      
    int block_size; //벽돌 크기

    bool red_event_handled = true; // 빨간선 처리 플래그 
    bool sign_event_handled = true; // 표지판 처리 플래그 
    bool parking_flag= false; //주차하러 가는 길 플래그
    //bool gate_flag= false; //차단기 앞에 도착 플래그
    bool prev_psd_done = false; //이전 psd 상태 저장 변수
    bool psd_sequence_done = false; //psd 밀기 시퀀스 완료 플래그
    bool wait_gate_flag= false; //차단기 대기 플래그
    bool block_flag= false; //벽돌 앞 도착 플래그


    void give_command(); //명령어 전달 함수
    void stop();
    void turn_left();
    void turn_right();
    void wait_until_gate_opens();
    void go_around_block();
    void parking();
    
   public:
    master();
    void publish_data();
    void callback(const custom_interfaces::msg::PacketProtocol::SharedPtr msg);//메시지를 받았을 때 호출할 콜백함수
};