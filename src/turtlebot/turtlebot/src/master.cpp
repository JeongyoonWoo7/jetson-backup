#include "turtlebot/master.hpp"


void master::callback(const custom_interfaces::msg::PacketProtocol::SharedPtr msg){ // 메시지를 받았을 때 호출할 콜백함수
    start= msg->start;
    red_flag = msg->red_flag;
    red_co.clear();
    red_co.push_back(msg->red_co[0]);
    red_co.push_back(msg->red_co[1]);
    yellow_flag = msg->yellow_flag;
    yellow_co.clear();
    yellow_co.push_back(msg->yellow_co[0]);
    yellow_co.push_back(msg->yellow_co[1]);
    white_flag = msg->white_flag;
    white_co.clear();
    white_co.push_back(msg->white_co[0]);
    white_co.push_back(msg->white_co[1]);
    traffic_flag = msg->traffic_flag;
    traffic_color = msg->traffic_color;
    sign_flag = msg->sign_flag;
    sign_co.clear();
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
        "vision_topic", 10, std::bind(&master::callback, this, std::placeholders::_1)); //서브스크립션 포인터 생성, 토픽명 설정, 콜백함수 바인딩

    sub_psd = this->create_subscription<std_msgs::msg::Bool>("psd_topic", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg){
        // false → true 변화 감지 (엣지 감지)
        if (prev_psd_done && msg->data) {
            this->psd_sequence_done = false;  // 변화 순간에만 true로 인식

        }
        else if(!prev_psd_done&& msg->data){
            this->psd_sequence_done= true;
        }  
        prev_psd_done = msg->data; // 상태 업데이트
    });
        // sub_int = this->create_subscription<std_msgs::msg::Int32>(
    //     "int_topic", 10, [](const std_msgs::msg::Int32::SharedPtr msg){
    //         start = msg->data;
    //     }); //int형 메시지 받는 서브스크립션 포인터 생성, 토픽명 설정, 람다식으로 콜백함수 정의
    pub = this->create_publisher<std_msgs::msg::String>("drive_topic", 10); //퍼블리시 포인터 생성, 토픽명 설정
    timer_= this->create_wall_timer(0.01s, std::bind(&master::give_command, this)); //타이머 생성, 강의자료 예제 코드 사용
    }

void master:: give_command(){ //명령어 전달 함수
    //여기에 명령어 전달을 위한 코드 작성
    std_msgs::msg::String msg;
    if(start==1){ //초반부에 시작하는 순간 이건 항상 계속 1로 둠 
        if(red_event_handled&&red_flag==1 && red_co[0]>=320 && red_co[1]>=220){ //빨간선이 감지되고, 좌표가 특정 값 이상일 때
            stop();
            return; //이건 맞음
        }
        //표지판 사이즈 넣을 수도 있음, 그리고 왼쪽 오른쪽으로 갔을 때 다시 왼쪽 오른쪽 가는 거 구현 누가??
        if(sign_event_handled&&sign_flag==1&& sign_co[0]>= 120&&sign_co[0]<=320 && sign_co[1]>=100&&sign_co[1]<=220){ //표지판이 감지되고, 좌표가 특정 값 이상일 때
            if(what_sign==1){
                turn_left();
                sign_event_handled=false;
                block_flag=true;
            }
            else if(what_sign==2){
                turn_right();
                sign_event_handled=false;
                block_flag=true;
            }
            return;
            
        }
        if(block==1&& block_size>=20){
            go_around_block(); //go_around_block 명령어가 주어지면 turn_left,psd처럼 다른 토픽이 와도 무시하고 계속 하던 일 해야 함
            parking_flag=true;
            block_flag=false;
            return;
        }
        if(parking_flag && yellow_flag==0){ //노란색 가로선 위치 잘 정해야..

            turn_left();
            parking();
            return;
        }
        if(gate==1){ //psd 관련 조건도 달아야 함 psd 관련 값을 subscribe로 받든가 해서
            wait_until_gate_opens(); //gate_flag 넣는 것은 디버깅할 때 문제 생기면 넣기 현재는 보류
            
            return;
        }
//        if( msg.data !="psd" &&msg.data != "stop"){ //기다리거나 psd 명령어가 아닐 때만 진행
            msg.data="go";
            pub->publish(msg);
//        }
    }
    
}
void master:: stop(){
    if(traffic_flag==1){
        std_msgs::msg::String msg;
        
        if(traffic_color==1 || traffic_color==2){ //빨간불, 노란불
            msg.data="stop";
            pub->publish(msg);
            return;
        }
        else{ //초록불
            msg.data="go";
            pub->publish(msg);
            red_event_handled=false; //빨간선 처리 완료
            red_flag=0;
            return;
        }


    }
    return;
}
void master:: turn_left(){ //turn left/right의 경우 다른 토픽 들어와도 무시하고 90도 회전하게 만들어야 함
    std_msgs::msg::String msg;
    msg.data="turn_left";
    pub->publish(msg);
    return;
}
void master:: turn_right(){
    std_msgs::msg::String msg;
    msg.data="turn_right";
    pub->publish(msg);
    return;
}
void master:: wait_until_gate_opens(){
    std_msgs::msg::String msg;

    msg.data="gate"; //이거는 자율주행 알고리즘이 앞 psd 값을 주기적으로 관찰하며 이동하도록 함. 
    pub->publish(msg);
    if(gate==0){ //차단기 열림
        msg.data="go";
        //gate_flag=false;
        pub->publish(msg);
        return;
    }
    return;
}
void master:: go_around_block(){ //아직 미완성
    std_msgs::msg::String msg;
    msg.data="go_around_block";
    pub->publish(msg);
    return;
}
void master:: parking(){
    static bool last_white_flag = white_flag;
    std_msgs::msg::String msg;
    msg.data="go";
    pub->publish(msg);
    
    if((!last_white_flag && white_flag) ||(last_white_flag && !white_flag)){ 
        msg.data= "stop";
        pub->publish(msg);
        pub->publish(msg);
        pub->publish(msg);
        msg.data="psd";
        pub->publish(msg);
        pub->publish(msg);//혹시나 놓칠까봐 제발 알아먹으라고 하는 것
        pub->publish(msg); 
        parking_flag=false;
        //gate_flag=true;
        return;

    } //흰색을 박스쳐서 그거 전체를 흰색으로 인식하게 하면 가능 그냥 색으로 걸리는 것만 할 경우 노이즈로 모두 검은색으로 인식할 수도

       last_white_flag = white_flag;
        return;
    //그 후 딜레이로 밀기 시퀀스적으로 기계적으로
 
    // while(!(!psd_sequence_done&& white_flag==0));

    // msg.data="turn_left";
    // pub->publish(msg);
    // psd_sequence_done=false;
 
}