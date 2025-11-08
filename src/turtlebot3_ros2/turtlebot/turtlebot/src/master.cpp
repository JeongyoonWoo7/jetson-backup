#include "turtlebot/master.hpp"


void master::callback(const custom_interfaces::msg::PacketProtocol::SharedPtr msg){ // 메시지를 받았을 때 호출할 콜백함수
    start= msg->start;
    red_flag = msg->red_flag;
    red_co.push_back(msg->red_co[0]);
    red_co.push_back(msg->red_co[1]);
    yellow_flag = msg->yellow_flag;
    yellow_co.push_back(msg->yellow_co[0]);
    yellow_co.push_back(msg->yellow_co[1]);
    white_flag = msg->white_flag;
    white_co.push_back(msg->white_co[0]);
    white_co.push_back(msg->white_co[1]);
    traffic_flag = msg->traffic_flag;
    traffic_color = msg->traffic_color;
    sign_flag = msg->sign_flag;
    sign_co.push_back(msg->sign_co[0]);
    sign_co.push_back(msg->sign_co[1]); 
    what_sign = msg->what_sign;
    gate = msg->gate;
    block = msg->block;
    block_size = msg->block_size;
  
}
master::master() : Node("master") //노드명 설정
{
    sub = this->create_subscription<custom_interfaces::msg::PacketProtocol>(
        "topic", 10, std::bind(&master::callback, this, std::placeholders::_1)); //서브스크립션 포인터 생성, 토픽명 설정, 콜백함수 바인딩
    // sub_int = this->create_subscription<std_msgs::msg::Int32>(
    //     "int_topic", 10, [](const std_msgs::msg::Int32::SharedPtr msg){
    //         start = msg->data;
    //     }); //int형 메시지 받는 서브스크립션 포인터 생성, 토픽명 설정, 람다식으로 콜백함수 정의
    pub = this->create_publisher<std_msgs::msg::String>("topic", 10); //퍼블리시 포인터 생성, 토픽명 설정
    timer_= this->create_wall_timer(0.01s, std::bind(&master::give_command, this)); //타이머 생성, 강의자료 예제 코드 사용
    }

void master:: give_command(){ //명령어 전달 함수
    //여기에 명령어 전달을 위한 코드 작성
    if(start==1){
        if(red_event_handled&&red_flag==1 && red_co[0]>=320 && red_co[1]>=220){ //빨간선이 감지되고, 좌표가 특정 값 이상일 때
            stop();
            return; 
            
        }
        if(sign_event_handled&&sign_flag==1&& sign_co[0]>= 120&&sign_co[0]<=320 && sign_co[1]>=100&&sign_co[1]<=220){ //표지판이 감지되고, 좌표가 특정 값 이상일 때
            if(what_sign==1){
                turn_left();
                sign_event_handled=false;
            }
            else if(what_sign==2){
                turn_right();
                sign_event_handled=false;
            }
            return;
        }
        if(block==1&& block_size>=20){
            go_around_block();
            parking_flag=true;
            return;
        }
        if(parking_flag)
        if(gate==1){
            wait_until_gate_opens();
            return;
        }

        std_msgs::msg::String msg;
        msg.data="go";
        pub->publish(msg);
    }
    
}
void master:: stop(){
    if(traffic_flag==1){
        std_msgs::msg::String msg;
        
        if(traffic_color==1 || traffic_color==2){ //빨간불, 노란불
            msg.data="stop";
            pub->publish(msg);
            
        }
        else{ //초록불
            msg.data="go";
            pub->publish(msg);
            red_event_handled=false; //빨간선 처리 완료
            red_flag=0;
        }


    }
}
void master:: turn_left(){
    std_msgs::msg::String msg;
    msg.data="turn_left";
    pub->publish(msg);
}
void master:: turn_right(){
    std_msgs::msg::String msg;
    msg.data="turn_right";
    pub->publish(msg);
    
}
void master:: wait_until_gate_opens(){
    std_msgs::msg::String msg;
    msg.data="wait";
    pub->publish(msg);
}
void master:: go_around_block(){
    std_msgs::msg::String msg;
    msg.data="go_around_block";
    pub->publish(msg);
}
