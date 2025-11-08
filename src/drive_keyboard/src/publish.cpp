#include "hw3.hpp"
int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<publisher>();     //노드 객체 생성
    node->publish_message(); //퍼블리셔 함수 호출
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}