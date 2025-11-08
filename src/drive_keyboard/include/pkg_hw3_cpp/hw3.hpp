#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp> //토픽 타입을 사용하기 위해 헤더 선언
#include <std_msgs/msg/string.hpp>
class publisher: public rclcpp::Node{
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub; //퍼블리셔 객체 선언
    rclcpp::TimerBase::SharedPtr timer_;    
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;
    char ch;
    public:
    publisher(); 
    void publish_message(); //퍼블리셔 함수 선언
    
};
int getch();