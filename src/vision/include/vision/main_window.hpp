#pragma once

#include <QMainWindow>
#include <QImage>
#include <QDebug>
#include <QSlider>
#include <QCheckBox>
#include <QPushButton>
#include <QPainter>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "ui_main_ui.h"
#include "ui_move_ui.h"   // move_ui (auto_frame) UI

#include <memory>
#include <chrono>
#include <QTimer>
#include <array>
#include <QDateTime> 

#include <mutex>
#include <opencv2/opencv.hpp>

#include "custom_interfaces/msg/packet_protocol.hpp"


class MainWindow : public QMainWindow, public std::enable_shared_from_this<MainWindow>
{
    Q_OBJECT

public:
    explicit MainWindow(rclcpp::Node::SharedPtr node, QWidget *parent = nullptr);
    ~MainWindow();

signals:
    void newImageAvailable(const QImage &img);

private slots:
    // === HSV 슬라이더 ===
    void onMoveHLowChanged(int value);
    void onMoveHHighChanged(int value);
    void onMoveSLowChanged(int value);
    void onMoveSHighChanged(int value);
    void onMoveKLowChanged(int value);
    void onMoveKHighChanged(int value);

    // === HSV 체크박스 ===
    void onMoveWhiteLineCheckChanged(bool checked);
    void onMoveYellowLineCheckChanged(bool checked);
    void onMoveRedLineLCheckChanged(bool checked);
    void onMoveRedLineHCheckChanged(bool checked);
    void onMoveBlockCheckChanged(bool checked);
    void onMoveGateCheckChanged(bool checked);
    void onMoveTrafficCheckChanged(bool checked);
    
    void onMoveTrafficRedCheckChanged(bool checked);
    void onMoveTrafficYellowCheckChanged(bool checked);
    void onMoveTrafficGreenCheckChanged(bool checked);

    // === 기타 UI 동작 ===
    void onHSVResetClicked();
    void onMoveStartClicked();
    void onMoveStopClicked();
    void startButtonClicked();
    void updateImage(const QImage &img);

    void onMoveRedChanged(int value);
    void onMoveGateChanged(int value);
    void onMoveLineChanged(int value);

    void onBXH_Changed(int value);
    void onBXL_Changed(int value);
    void onBYH_Changed(int value);
    void onBYL_Changed(int value);

private:
    void updateMask(cv::Mat &mask, const cv::Scalar &lower, const cv::Scalar &upper, QLabel *label);

    // === ROS 이미지 콜백 ===
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    // 가로선
    void drawHorizontalLineOnLabel(QLabel *label, int y_value, const QColor &color, int thickness);

    // bbox 처리 및 그리기
    void processAndDrawBBox(const cv::Scalar &lower, const cv::Scalar &upper,
                            const QColor &penColor, QLabel *maskLabel, QLabel *drawLabel,
                            const QString &logName, const std::array<int,4> &roi, std::array<int,4> &out_bbox);
    
    // 버드 아이즈
    std::array<cv::Point2f, 4> computeBirdsEyeQuad();

    // ===== 차선 검출 및 발행 =====
    void findLaneAndPublish(const cv::Mat &yellow_mask, const cv::Mat &white_mask);
    void publishLaneInfo(float offset_m, float angle, bool left_found, bool right_found);

    // ===== 퍼블리셔 =====
    rclcpp::Publisher<custom_interfaces::msg::PacketProtocol>::SharedPtr lane_pub_;

    QTimer* gui_timer_;
    std::shared_ptr<QImage> latest_image_;
    int gui_fps_ = 180;        

    // === UI 포인터 ===
    Ui::MainUI *ui;               // 메인 화면
    Ui::auto_frame *moveUi;       // 주행 (move_ui)
    QMainWindow *moveWidget;      // 주행용 QMainWindow

    // === ROS 관련 ===
    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cam_sub_;

    // === HSV 색상 범위 ===
    cv::Scalar lower_white_line, upper_white_line;
    cv::Scalar lower_yellow_line, upper_yellow_line;
    cv::Scalar lower_red_line_low, upper_red_line_low;
    cv::Scalar lower_red_line_high, upper_red_line_high;
    cv::Scalar lower_block, upper_block;
    cv::Scalar lower_gate, upper_gate;
    cv::Scalar lower_traffic, upper_traffic;
    cv::Scalar lower_sign, upper_sign;
    cv::Scalar lower_traffic_red, upper_traffic_red;
    cv::Scalar lower_traffic_yellow, upper_traffic_yellow;
    cv::Scalar lower_traffic_green, upper_traffic_green;

    // 마스크 생성용 멤버(스레드 안전)
    cv::Mat img_hsv_;
    std::mutex img_mutex_;

    // === 사용 여부 플래그 ===
    bool use_white_line = false;
    bool use_yellow_line = false;
    bool use_red_line_H = false;
    bool use_red_line_L = false;
    bool use_block = false;
    bool use_gate = false;
    bool use_traffic = false;
    bool use_sign = false;
    bool use_traffic_red = false;
    bool use_traffic_yellow = false;
    bool use_traffic_green = false;

    int line_y;
    int gate_y;
    int red_y;

    // ROI 배열들 (원본 이미지 기준) -> {x, y, w, h}
    std::array<int,4> roi_white_line{0,0,0,0};
    std::array<int,4> roi_yellow_line{0,0,0,0};
    std::array<int,4> roi_red_line{0,0,0,0};
    std::array<int,4> roi_block{0,0,0,0};
    std::array<int,4> roi_gate{0,0,0,0};
    std::array<int,4> roi_traffic{0,0,0,0};

    // 찾은 바운딩박스 저장용 배열들 -> {x, y, w, h}, 찾지 못하면 {-1,-1,0,0}
    std::array<int,4> bbox_white_line{ -1,-1,0,0 };
    std::array<int,4> bbox_yellow_line{ -1,-1,0,0 };
    std::array<int,4> bbox_red_line{ -1,-1,0,0 };
    std::array<int,4> bbox_block{ -1,-1,0,0 };
    std::array<int,4> bbox_gate{ -1,-1,0,0 };
    std::array<int,4> bbox_traffic{ -1,-1,0,0 };

    cv::Mat img_bgr_;

    // ROI / Bird's-eye 관련 함수 선언
    // bbox: {x,y,w,h} 원본 기준
    void showROI(const std::array<int,4> &bbox, QLabel *roiLabel, QLabel *drawLabel = nullptr, const QColor &penColor = Qt::yellow);

    void showBirdsEye(const std::array<cv::Point2f,4> &srcQuad, const cv::Size &dstSize, QLabel *outLabel, bool drawBorderOnSrc = false, const QColor &borderColor = Qt::green);
    
    int birdseye_y_high = 0;
    int birdseye_y_low = 0;
    int birdseye_x_high = 0;
    int birdseye_x_low = 0;

    std::mutex bev_mutex_;
    cv::Mat bev_bgr_;
    
    // 마스크 저장용 (white/yellow line)
    cv::Mat yellow_line_mask_;
    cv::Mat white_line_mask_;
    cv::Mat red_line_mask_;

    // 시작여부
    int is_started_ = 0;

    bool left_found_cord = false;
    bool right_found_cord = false;
    float offset_cord = 0.0f;
    float angle_cord = 0.0f;

    // 기타 정보
    bool red_flag = false;
    bool traffic_flag = false;
    int traffic_color_val = 0;
    bool sign_flag = false;
    int sign_size_val = 0;
    int sign_type = 0;
    bool gate_detected = false;
    bool block_detected = false;
    bool turn_flag = false;

};
