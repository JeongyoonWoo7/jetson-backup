#include "hw3.hpp"
#include <iostream>
#include <chrono>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>

// C의 getch() 대체
int getch(void)
{
    struct termios oldattr, newattr;
    int ch;
    tcgetattr(STDIN_FILENO, &oldattr);
    newattr = oldattr;
    newattr.c_lflag &= ~(ICANON | ECHO); // 입력 버퍼 비활성화 + 에코 끔
    tcsetattr(STDIN_FILENO, TCSANOW, &newattr);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldattr);
    return ch;
}

publisher::publisher():Node("publish_node"){ //노드명 설정
    pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10); //해당 토픽명으로 퍼블리셔 생성
    // timer_ = this->create_wall_timer( //타이머 생성
    //     std::chrono::milliseconds(20),
    //     std::bind(&publisher::publish_message,this) //500ms마다 publish_message 함수 호출

    // );

// sub = this->create_subscription<std_msgs::msg::String>(
//     "key_input", 10,
//     [this](const std_msgs::msg::String::SharedPtr msg){
//         RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
//        this-> ch = msg->data[0];
//     });
    ch = ' ';
}
void publisher::publish_message(){
    while(rclcpp::ok()){
     ch = getch(); //키보드 입력 받기

     if(ch == 'W'|| ch=='w'){ //w 입력시 정사각형 그림
       
        geometry_msgs::msg::Twist temp; //메시지 타입 객체 생성
        
        temp.linear.x=0.3;
        temp.angular.z=0.0;
        pub-> publish(temp); 
        
        

     
    }
     else if(ch== 'A'||ch=='a'){ // 세모 그림
      
        geometry_msgs::msg::Twist temp;
         temp.linear.x=0.0;   
         temp.angular.z=0.8;
         pub-> publish(temp);

    }
    
     else if(ch== 'S'||ch=='s'){//원 그림
         geometry_msgs::msg::Twist temp;
        temp.linear.x=-0.3;
        temp.angular.z=0.0;
        pub-> publish(temp); 

        
      }
     else if(ch== 'D'||ch=='d'){ //지금까지 그린 것 삭제
         geometry_msgs::msg::Twist temp;
          temp.linear.x=0.0;   
          temp.angular.z=-0.8;
          pub-> publish(temp);
     
     }
     else if(ch== 'z'||ch=='Z'){ //정지
         geometry_msgs::msg::Twist temp;
         temp.linear.x=0.0;   
         temp.angular.z=0.0;
         pub-> publish(temp);
     }
     
    }

}