#include "vision/main_window.hpp"
#include <QApplication>
#include <rclcpp/rclcpp.hpp>
#include <thread>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("vision_node");

    QApplication app(argc, argv);
    MainWindow window(node); // 스택 객체
    window.show();


    std::thread ros_thread([node]() {
        rclcpp::spin(node);
    });

    int ret = app.exec();

    rclcpp::shutdown();
    ros_thread.join();

    return ret;
}

