#include "vision/main_window.hpp"

MainWindow::MainWindow(rclcpp::Node::SharedPtr node, QWidget *parent)
    : QMainWindow(parent),
      ui(new Ui::MainUI),
      moveUi(nullptr),
      moveWidget(nullptr),
      ros_node_(node)
{
    ui->setupUi(this);

    // "시작 버튼" 클릭 시 move_ui 전환
    connect(ui->pushButton_5, &QPushButton::clicked, this, &MainWindow::startButtonClicked);

    // ROS 이미지 수신 시그널 연결
    connect(this, &MainWindow::newImageAvailable, this, &MainWindow::updateImage);

    lane_pub_ = ros_node_->create_publisher<custom_interfaces::msg::PacketProtocol>("/packet_protocol", 10);

}

MainWindow::~MainWindow()
{
    if (cam_sub_) cam_sub_.reset();
    disconnect(this, &MainWindow::newImageAvailable, this, &MainWindow::updateImage);

    delete moveUi;
    delete moveWidget;
    delete ui;
}

void MainWindow::startButtonClicked()
{
    if (!moveWidget)
    {
        moveWidget = new QMainWindow(this);
        moveUi = new Ui::auto_frame();
        moveUi->setupUi(static_cast<QMainWindow*>(moveWidget));

        moveUi->label_4->setText(QStringLiteral("stop"));
        moveUi->label_5->setText(QStringLiteral("traffic"));

        is_started_ = 0;

        // 가로선 슬라이더 범위
        moveUi->red_line_Y->setRange(0, 480);
        moveUi->red_line_Y->setInvertedAppearance(true);
        moveUi->red_line_Y->setInvertedControls(true);

        moveUi->gate_line_Y->setRange(0, 480);
        moveUi->gate_line_Y->setInvertedAppearance(true);
        moveUi->gate_line_Y->setInvertedControls(true);

        moveUi->line_Y->setRange(0, 480);
        moveUi->line_Y->setInvertedAppearance(true);
        moveUi->line_Y->setInvertedControls(true);

        // 가로선 슬라이더 연결
        connect(moveUi->red_line_Y,  &QSlider::valueChanged, this, &MainWindow::onMoveRedChanged);
        connect(moveUi->gate_line_Y, &QSlider::valueChanged, this, &MainWindow::onMoveGateChanged);
        connect(moveUi->line_Y,  &QSlider::valueChanged, this, &MainWindow::onMoveLineChanged);

        // 가로선 위치 초기값
        line_y = 240;
        gate_y = 240;
        red_y = 240;

        // 가로선 슬라이더 연동
        moveUi->red_line_Y->setValue(line_y);
        moveUi->gate_line_Y->setValue(gate_y);
        moveUi->line_Y->setValue(red_y);

        // 버드아이즈 슬라이더 범위
        moveUi->birdseye_x_high_slider->setRange(0, 320);
        moveUi->birdseye_x_low_silder->setRange(0, 320);

        moveUi->birdseye_y_high_slider->setRange(0, 480);
        moveUi->birdseye_y_high_slider->setInvertedAppearance(true);
        moveUi->birdseye_y_high_slider->setInvertedControls(true);

        moveUi->birdseye_y_low_slider->setRange(0, 480);
        moveUi->birdseye_y_low_slider->setInvertedAppearance(true);
        moveUi->birdseye_y_low_slider->setInvertedControls(true);

        // 버드아이즈 슬라이더 연결
        connect(moveUi->birdseye_x_high_slider,  &QSlider::valueChanged, this, &MainWindow::onBXH_Changed);
        connect(moveUi->birdseye_x_low_silder, &QSlider::valueChanged, this, &MainWindow::onBXL_Changed);
        connect(moveUi->birdseye_y_high_slider,  &QSlider::valueChanged, this, &MainWindow::onBYH_Changed);
        connect(moveUi->birdseye_y_low_slider,  &QSlider::valueChanged, this, &MainWindow::onBYL_Changed);

        // 버드아이즈 위치 초기값
        birdseye_y_high = 280;
        birdseye_y_low = 410;
        birdseye_x_high = 180;
        birdseye_x_low = 320;

        // 버드아이즈 슬라이더 연동
        moveUi->birdseye_x_high_slider->setValue(birdseye_x_high);
        moveUi->birdseye_x_low_silder->setValue(birdseye_x_low);
        moveUi->birdseye_y_high_slider->setValue(birdseye_y_high);
        moveUi->birdseye_y_low_slider->setValue(birdseye_y_low);
        
        // HSV 슬라이더 범위
        moveUi->H_low_slider->setRange(0, 179);
        moveUi->H_high_slider->setRange(0, 179);
        moveUi->S_low_slider->setRange(0, 255);
        moveUi->S_high_slider->setRange(0, 255);
        moveUi->K_low_slider->setRange(0, 255);
        moveUi->K_high_slider->setRange(0, 255);

        // HSV 슬라이더 연결
        connect(moveUi->H_low_slider,  &QSlider::valueChanged, this, &MainWindow::onMoveHLowChanged);
        connect(moveUi->H_high_slider, &QSlider::valueChanged, this, &MainWindow::onMoveHHighChanged);
        connect(moveUi->S_low_slider,  &QSlider::valueChanged, this, &MainWindow::onMoveSLowChanged);
        connect(moveUi->S_high_slider, &QSlider::valueChanged, this, &MainWindow::onMoveSHighChanged);
        connect(moveUi->K_low_slider,  &QSlider::valueChanged, this, &MainWindow::onMoveKLowChanged);
        connect(moveUi->K_high_slider, &QSlider::valueChanged, this, &MainWindow::onMoveKHighChanged);

        // 체크박스 연결
        connect(moveUi->white_line_2,  &QCheckBox::toggled, this, &MainWindow::onMoveWhiteLineCheckChanged);
        connect(moveUi->yellow_line_2, &QCheckBox::toggled, this, &MainWindow::onMoveYellowLineCheckChanged);
        connect(moveUi->red_line_low,  &QCheckBox::toggled, this, &MainWindow::onMoveRedLineLCheckChanged);
        connect(moveUi->red_line_high, &QCheckBox::toggled, this, &MainWindow::onMoveRedLineHCheckChanged);
        connect(moveUi->block_2,       &QCheckBox::toggled, this, &MainWindow::onMoveBlockCheckChanged);
        connect(moveUi->gate_2,        &QCheckBox::toggled, this, &MainWindow::onMoveGateCheckChanged);
        connect(moveUi->traffic_2,     &QCheckBox::toggled, this, &MainWindow::onMoveTrafficCheckChanged);
        connect(moveUi->traffic_red,   &QCheckBox::toggled, this, &MainWindow::onMoveTrafficRedCheckChanged);
        connect(moveUi->traffic_yellow,&QCheckBox::toggled, this, &MainWindow::onMoveTrafficYellowCheckChanged);
        connect(moveUi->traffic_green, &QCheckBox::toggled, this, &MainWindow::onMoveTrafficGreenCheckChanged);

        // HSV 초기값
        lower_white_line  = cv::Scalar(0, 0, 133);
        upper_white_line  = cv::Scalar(157, 29, 255);
        lower_yellow_line = cv::Scalar(10, 40, 111);
        upper_yellow_line = cv::Scalar(90, 255, 255);
        lower_red_line_low    = cv::Scalar(0, 8, 32);
        upper_red_line_low    = cv::Scalar(8, 255, 255);
        lower_red_line_high   = cv::Scalar(169, 4, 0);
        upper_red_line_high   = cv::Scalar(179, 255, 255);
        lower_block       = cv::Scalar(100, 100, 100);
        upper_block       = cv::Scalar(130, 255, 255);
        lower_gate        = cv::Scalar(0, 120, 80);
        upper_gate        = cv::Scalar(6, 255, 255);
        lower_traffic     = cv::Scalar(0, 50, 50);
        upper_traffic     = cv::Scalar(10, 255, 255);
        lower_traffic_red       = cv::Scalar(0, 50, 50);
        upper_traffic_red       = cv::Scalar(10, 255, 255);
        lower_traffic_yellow    = cv::Scalar(0, 50, 50);
        upper_traffic_yellow    = cv::Scalar(10, 255, 255);
        lower_traffic_green     = cv::Scalar(0, 50, 50);
        upper_traffic_green     = cv::Scalar(10, 255, 255);

        // 상태 초기화
        use_white_line      = true;
        use_yellow_line     = false;
        use_red_line_L      = false;
        use_red_line_H      = false;
        use_block           = false;
        use_gate            = false;
        use_traffic         = false;
        use_sign            = false;
        use_traffic_red     = false;
        use_traffic_yellow  = false;
        use_traffic_green   = false;

        // 기본 선택
        moveUi->white_line_2->setChecked(true);

        // HSV 리셋 버튼
        connect(moveUi->hsv_reset, &QPushButton::clicked, this, &MainWindow::onHSVResetClicked);

        // 주행 버튼
        connect(moveUi->start, &QPushButton::clicked, this, &MainWindow::onMoveStartClicked);
        connect(moveUi->stop,  &QPushButton::clicked, this, &MainWindow::onMoveStopClicked);

        // ===== ROS 이미지 구독 =====
        cam_sub_ = ros_node_->create_subscription<sensor_msgs::msg::Image>(
            "/camera1/image_raw",
            10,
            [this](const sensor_msgs::msg::Image::SharedPtr msg){ this->imageCallback(msg); }
        );

        // ===== ROS 시그널 연결 ===== -> 이미 생성자에서 연결되어 있으므로 중복 연결하지 않음
        //connect(this, &MainWindow::newImageAvailable, this, &MainWindow::updateImage);
    }

    this->hide();
    moveWidget->show();
}

// 차선 검출 및 퍼블리시
void MainWindow::findLaneAndPublish(const cv::Mat &yellow_mask, const cv::Mat &white_mask)
{
    if (yellow_mask.empty() && white_mask.empty()) return;

    int img_center = yellow_mask.cols / 2;

    // 왼쪽(노란선) 포인트
    std::vector<std::vector<cv::Point>> yellow_contours;
    cv::findContours(yellow_mask, yellow_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::vector<cv::Point> left_pts;
    for (auto &contour : yellow_contours) {
        if (cv::contourArea(contour) < 200) continue;
        for (auto &p : contour) left_pts.push_back(p);
    }

    // 오른쪽(흰선) 포인트
    std::vector<std::vector<cv::Point>> white_contours;
    cv::findContours(white_mask, white_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::vector<cv::Point> right_pts;
    for (auto &contour : white_contours) {
        if (cv::contourArea(contour) < 200) continue;
        for (auto &p : contour) right_pts.push_back(p);
    }

    // yellow OR white
    cv::Mat mask_bev;
    if (!yellow_mask.empty() && !white_mask.empty()) {
        cv::bitwise_or(yellow_mask, white_mask, mask_bev);
    } else if (!yellow_mask.empty()) {
        mask_bev = yellow_mask.clone();
    } else if (!white_mask.empty()) {
        mask_bev = white_mask.clone();
    }

    // cord 윤곽선 검출
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask_bev, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    int img_center_cord = mask_bev.cols / 2;
    std::vector<cv::Point> left_pts_cord, right_pts_cord;

    for (auto &contour : contours) {
        cv::Rect bbox = cv::boundingRect(contour);
        if (bbox.area() < 200) continue;
        cv::Moments M = cv::moments(contour);
        if (M.m00 == 0) continue;
        int cx = static_cast<int>(M.m10 / M.m00);
        int cy = static_cast<int>(M.m01 / M.m00);
        if (cx < img_center_cord) left_pts_cord.push_back({cx, cy});
        else right_pts_cord.push_back({cx, cy});
    }

    // ===== GUI 표시 =====
    if (moveUi && moveUi->yellow_con) {
        QPixmap pm = moveUi->yellow_con->pixmap(Qt::ReturnByValue);
        if (!pm.isNull()) {
            QPixmap pix = pm.copy();
            QPainter painter(&pix);
            painter.setRenderHint(QPainter::Antialiasing, true);

            double sx = double(pix.width())  / double(yellow_mask.cols);
            double sy = double(pix.height()) / double(yellow_mask.rows);

            painter.setBrush(Qt::yellow);
            painter.setPen(Qt::NoPen);
            for (const auto &p : left_pts)
                painter.drawEllipse(QPoint(p.x * sx, p.y * sy), 4, 4);

            painter.setBrush(Qt::white);
            for (const auto &p : right_pts)
                painter.drawEllipse(QPoint(p.x * sx, p.y * sy), 4, 4);

            // 왼쪽 포인트(빨강)
            painter.setBrush(Qt::red);
            painter.setPen(Qt::NoPen);
            for (const auto &p : left_pts_cord) {
                int px = std::max(0, std::min(pix.width()-1, static_cast<int>(std::round(p.x * sx))));
                int py = std::max(0, std::min(pix.height()-1, static_cast<int>(std::round(p.y * sy))));
                painter.drawEllipse(QPoint(px, py), 4, 4);
            }

            // 오른쪽 포인트(파랑)
            painter.setBrush(Qt::blue);
            for (const auto &p : right_pts_cord) {
                int px = std::max(0, std::min(pix.width()-1, static_cast<int>(std::round(p.x * sx))));
                int py = std::max(0, std::min(pix.height()-1, static_cast<int>(std::round(p.y * sy))));
                painter.drawEllipse(QPoint(px, py), 4, 4);
            }

            painter.end();
            moveUi->yellow_con->setPixmap(pix);
        }
    }

    left_found_cord = left_pts_cord.size() > 0;
    right_found_cord = right_pts_cord.size() > 0;

    cv::Vec4f left_line_cord, right_line_cord;
    if (left_found_cord)
        cv::fitLine(left_pts_cord, left_line_cord, cv::DIST_L2, 0, 0.01, 0.01);
    if (right_found_cord)
        cv::fitLine(right_pts_cord, right_line_cord, cv::DIST_L2, 0, 0.01, 0.01);

    float lane_center_x_cord = img_center_cord;
    const float M_PER_PX = 0.000588f;
    const float LANE_WIDTH_M = 0.2f;

    // 3. 중심 계산
    if (left_found_cord && right_found_cord) {
        float x_left_bottom_cord = left_line_cord[2] + (mask_bev.rows - left_line_cord[3]) * (left_line_cord[0] / left_line_cord[1]);
        float x_right_bottom_cord = right_line_cord[2] + (mask_bev.rows - right_line_cord[3]) * (right_line_cord[0] / right_line_cord[1]);
        lane_center_x_cord = (x_left_bottom_cord + x_right_bottom_cord) / 2.0f;
    } else if (left_found_cord) {
        float x_left_bottom_cord = left_line_cord[2] + (mask_bev.rows - left_line_cord[3]) * (left_line_cord[0] / left_line_cord[1]);
        lane_center_x_cord = x_left_bottom_cord + (LANE_WIDTH_M / M_PER_PX) / 2.0f;
    } else if (right_found_cord) {
        float x_right_bottom_cord = right_line_cord[2] + (mask_bev.rows - right_line_cord[3]) * (right_line_cord[0] / right_line_cord[1]);
        lane_center_x_cord = x_right_bottom_cord - (LANE_WIDTH_M / M_PER_PX) / 2.0f;
    }

    // 4. 오프셋 및 각도 계산 (BEV 기준이라 실제 차선 기울기에 근접)
    float offset_px_cord = lane_center_x_cord - img_center_cord;
    offset_cord = offset_px_cord * M_PER_PX;

    angle_cord = 0.0f;
    if (left_found_cord && right_found_cord) {
        float angle_left_cord = atan2(left_line_cord[1], left_line_cord[0]);
        float angle_right_cord = atan2(right_line_cord[1], right_line_cord[0]);
        angle_cord = (angle_left_cord + angle_right_cord) / 2.0f;
    } else if (left_found_cord)
        angle_cord = atan2(left_line_cord[1], left_line_cord[0]);
    else if (right_found_cord)
        angle_cord = atan2(right_line_cord[1], right_line_cord[0]);

// -------------------------------------------------------------------        

    // 선 피팅
    cv::Vec4f left_line, right_line;
    bool left_found = left_pts.size() > 0;
    bool right_found = right_pts.size() > 0;

    if (left_found)
        cv::fitLine(left_pts, left_line, cv::DIST_L2, 0, 0.01, 0.01);
    if (right_found)
        cv::fitLine(right_pts, right_line, cv::DIST_L2, 0, 0.01, 0.01);

    // 중심/오프셋/각도 계산
    float lane_center_x = img_center;

    if (left_found && right_found) {
        float x_left_bottom = left_line[2] + (yellow_mask.rows - left_line[3]) * (left_line[0] / left_line[1]);
        float x_right_bottom = right_line[2] + (white_mask.rows - right_line[3]) * (right_line[0] / right_line[1]);
        lane_center_x = (x_left_bottom + x_right_bottom) / 2.0f;
    } else if (left_found) {
        float x_left_bottom = left_line[2] + (yellow_mask.rows - left_line[3]) * (left_line[0] / left_line[1]);
        lane_center_x = x_left_bottom + (LANE_WIDTH_M / M_PER_PX) / 2.0f;
    } else if (right_found) {
        float x_right_bottom = right_line[2] + (white_mask.rows - right_line[3]) * (right_line[0] / right_line[1]);
        lane_center_x = x_right_bottom - (LANE_WIDTH_M / M_PER_PX) / 2.0f;
    }

    float offset_px = lane_center_x - img_center;
    float offset_m = offset_px * M_PER_PX;

    float lane_angle = 0.0f;
    if (left_found && right_found) {
        float angle_left = atan2(left_line[1], left_line[0]);
        float angle_right = atan2(right_line[1], right_line[0]);
        lane_angle = (angle_left + angle_right) / 2.0f;
    } else if (left_found)
        lane_angle = atan2(left_line[1], left_line[0]);
    else if (right_found)
        lane_angle = atan2(right_line[1], right_line[0]);

    publishLaneInfo(offset_m, lane_angle, left_found, right_found);
}


void MainWindow::publishLaneInfo(float offset_m, float angle, bool left_found, bool right_found)
{
    if (!lane_pub_) return; // 퍼블리셔 생성 확인

    custom_interfaces::msg::PacketProtocol msg;
    
    // 1) 첫 번째 필드 start 설정
    msg.start = is_started_; // is_started_ = 0;

    // 2) 색상 기반 차선 정보
    msg.left_detected_color  = left_found;
    msg.right_detected_color = right_found;
    msg.lane_center_offset_color = offset_m;
    msg.lane_angle_color        = angle;

    // 3) 좌표 기반 차선 정보
    msg.left_detected_cord  = left_found_cord;
    msg.right_detected_cord = right_found_cord;
    msg.lane_center_offset_cord = offset_cord;
    msg.lane_angle_cord        = angle_cord;

    // 4) 기타 정보
    msg.red_flag      = red_flag;
    msg.traffic_flag  = traffic_flag;
    msg.traffic_color = traffic_color_val;
    msg.sign_flag     = sign_flag;
    msg.sign_size     = sign_size_val;
    msg.what_sign     = sign_type;
    msg.gate          = gate_detected;
    msg.block         = block_detected;
    msg.turn          = turn_flag;

    lane_pub_->publish(msg);

    // GUI 로그
    if (moveUi && moveUi->textEdit) {
        double angle_deg = angle * 180.0 / M_PI;
        QString summary = QString("Lane -> L:%1 R:%2  off:%3 m  ang:%4 deg")
            .arg(left_found ? "Y" : "N")
            .arg(right_found ? "Y" : "N")
            .arg(QString::number(offset_m, 'f', 3))
            .arg(QString::number(angle_deg, 'f', 2));

        QString detail = QString("time: %1 | offset:%2 m | angle:%3 deg | L:%4 R:%5")
            .arg(QDateTime::currentDateTime().toString("HH:mm:ss"))
            .arg(QString::number(offset_m, 'f', 3))
            .arg(QString::number(angle_deg, 'f', 2))
            .arg(left_found ? "Y" : "N")
            .arg(right_found ? "Y" : "N");

        int gate_size = bbox_gate[2] * bbox_gate[3];

        QString gate_log = QString("Gate:%1 | size:%2")
            .arg(gate_detected ? "Y" : "N")
            .arg(gate_size);

        int red_size = bbox_red_line[2] * bbox_red_line[3];
        int y_top = bbox_red_line[1];
        int y_bottom = bbox_red_line[1] + bbox_red_line[3];

        QString red_log = QString("Red Sign:%1 | size:%2 | y_top:%3 | y_bottom:%4 | %5")
            .arg(sign_flag ? "Y" : "N")
            .arg(red_size)
            .arg(y_top)
            .arg(y_bottom)
            .arg(red_y);

        moveUi->textEdit->setText(summary);     // 덮어쓰기 (상단 한줄)
        moveUi->textEdit->append(detail);      // 이력 추가
        moveUi->textEdit->append(gate_log);    // 게이트 상태 추가
        moveUi->textEdit->append(red_log);     // 정지 표지판 상태 추가
        moveUi->textEdit->append("-------------------------------");
    }
}

// ROS 이미지 콜백
void MainWindow::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    if (!msg) return;
    try {
        cv::Mat cv_img = cv_bridge::toCvCopy(msg, "bgr8")->image;
        if (cv_img.empty()) return;

        // HSV 먼저 생성
        cv::Mat hsv;
        cv::cvtColor(cv_img, hsv, cv::COLOR_BGR2HSV);

        // 원본 BGR 및 HSV를 한 번에 안전하게 저장 (올바른 뮤텍스: img_mutex_)
        {
            std::lock_guard<std::mutex> lk(img_mutex_);
            img_bgr_ = cv_img.clone();
            img_hsv_ = hsv.clone();
        }

        // 디버깅: BEV/원본 저장 상태 확인 로그
        RCLCPP_DEBUG(ros_node_->get_logger(), "imageCallback stored img_bgr_ %dx%d", img_bgr_.cols, img_bgr_.rows);

        // UI 전송용 RGB QImage 생성 (항상 RGB로 변환하고 깊은 복사)
        cv::Mat rgb;
        cv::cvtColor(cv_img, rgb, cv::COLOR_BGR2RGB);
        QImage qimg(reinterpret_cast<const uchar*>(rgb.data),
                    rgb.cols, rgb.rows, static_cast<int>(rgb.step),
                    QImage::Format_RGB888);
        emit newImageAvailable(qimg.copy());

        // 디버그 로깅 (선택)
        RCLCPP_DEBUG(ros_node_->get_logger(), "Received image size: %d x %d", cv_img.cols, cv_img.rows);

    } catch (const cv_bridge::Exception &e) {
        RCLCPP_ERROR(ros_node_->get_logger(), "cv_bridge exception: %s", e.what());
    } catch (const std::exception &e) {
        RCLCPP_ERROR(ros_node_->get_logger(), "exception in imageCallback: %s", e.what());
    }
}
/*
void MainWindow::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    if (!msg) return;
    try {
        cv::Mat cv_img = cv_bridge::toCvCopy(msg, "bgr8")->image;
        cv::Mat rgb;
        cv::cvtColor(cv_img, rgb, cv::COLOR_BGR2RGB);
        QImage qimg(rgb.data, rgb.cols, rgb.rows, rgb.step, QImage::Format_RGB888);
-        emit newImageAvailable(qimg);
+        // QImage가 rgb.data를 참조하므로 안전하게 깊은 복사 후 emit
+        emit newImageAvailable(qimg.copy());
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(ros_node_->get_logger(), "cv_bridge exception: %s", e.what());
    }
}*/

std::array<cv::Point2f,4> MainWindow::computeBirdsEyeQuad()
{
    int orig_w = 640, orig_h = 480;
    {
        std::lock_guard<std::mutex> lk(img_mutex_);
        if (!img_bgr_.empty()) { orig_w = img_bgr_.cols; orig_h = img_bgr_.rows; }
    }

    int cx = orig_w / 2;

    int topY    = std::clamp(birdseye_y_high,  0, orig_h - 1);
    int bottomY = std::clamp(birdseye_y_low,   0, orig_h - 1);
    int thw     = std::clamp(birdseye_x_high,  0, cx); // 윗변 중심에서의 반폭
    int bhw     = std::clamp(birdseye_x_low,   0, cx); // 아랫변 중심에서의 반폭

    int tlx = std::clamp(cx - thw, 0, orig_w - 1);
    int trx = std::clamp(cx + thw, 0, orig_w - 1);
    int blx = std::clamp(cx - bhw, 0, orig_w - 1);
    int brx = std::clamp(cx + bhw, 0, orig_w - 1);

    return { cv::Point2f((float)tlx, (float)topY),
             cv::Point2f((float)trx, (float)topY),
             cv::Point2f((float)brx, (float)bottomY),
             cv::Point2f((float)blx, (float)bottomY) };
}

void MainWindow::showROI(const std::array<int,4> &bbox, QLabel *roiLabel, QLabel *drawLabel, const QColor &penColor)
{
    // 안전 체크
    if (bbox[2] <= 0 || bbox[3] <= 0) return;

    cv::Mat orig;
    {
        std::lock_guard<std::mutex> lk(img_mutex_);
        if (img_bgr_.empty()) return;
        orig = img_bgr_.clone();
    }

    // clip
    int x = std::max(0, std::min(bbox[0], orig.cols-1));
    int y = std::max(0, std::min(bbox[1], orig.rows-1));
    int w = std::max(1, std::min(bbox[2], orig.cols - x));
    int h = std::max(1, std::min(bbox[3], orig.rows - y));

    // 크롭하여 roiLabel에 표시
    if (roiLabel) {
        cv::Mat crop = orig(cv::Rect(x,y,w,h)).clone();
        cv::Mat crop_rgb;
        cv::cvtColor(crop, crop_rgb, cv::COLOR_BGR2RGB);
        QImage qimg_crop((const uchar*)crop_rgb.data, crop_rgb.cols, crop_rgb.rows, static_cast<int>(crop_rgb.step), QImage::Format_RGB888);
        roiLabel->setPixmap(QPixmap::fromImage(qimg_crop.copy()).scaled(roiLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    }

    // drawLabel에 원본 위에 bbox 그리기 (매핑)
    if (drawLabel) {
        QPixmap pm = drawLabel->pixmap(Qt::ReturnByValue);
        if (pm.isNull()) return;
        QPixmap pix = pm.copy();

        int orig_w = orig.cols, orig_h = orig.rows;
        double sx = double(pix.width())  / double(orig_w);
        double sy = double(pix.height()) / double(orig_h);

        int rx = std::max(0, std::min(pix.width()-1, static_cast<int>(std::round(x * sx))));
        int ry = std::max(0, std::min(pix.height()-1, static_cast<int>(std::round(y * sy))));
        int rw = std::max(1, std::min(pix.width()-rx, static_cast<int>(std::round(w * sx))));
        int rh = std::max(1, std::min(pix.height()-ry, static_cast<int>(std::round(h * sy))));

        QPainter painter(&pix);
        QPen pen(penColor);
        pen.setWidth(2);
        painter.setPen(pen);
        painter.drawRect(rx, ry, rw, rh);
        painter.end();

        drawLabel->setPixmap(pix);
    }
}

// Bird's-eye view 생성 및 QLabel에 표시
void MainWindow::showBirdsEye(const std::array<cv::Point2f,4> &srcQuad, const cv::Size &dstSize, QLabel *outLabel, bool drawBorderOnSrc, const QColor &borderColor)
{
    if (!outLabel) return;

    cv::Mat orig;
    {
        std::lock_guard<std::mutex> lk(img_mutex_);
        if (img_bgr_.empty()) return;
        orig = img_bgr_.clone();
    }

    // 목적지 사각형 (dst 좌표계)
    std::array<cv::Point2f,4> dstQuad = { cv::Point2f(0,0), cv::Point2f((float)dstSize.width-1,0),
                                          cv::Point2f((float)dstSize.width-1,(float)dstSize.height-1),
                                          cv::Point2f(0,(float)dstSize.height-1) };

    cv::Mat M = cv::getPerspectiveTransform(std::vector<cv::Point2f>(srcQuad.begin(), srcQuad.end()),
                                            std::vector<cv::Point2f>(dstQuad.begin(), dstQuad.end()));
    cv::Mat bev;
    cv::warpPerspective(orig, bev, M, dstSize, cv::INTER_LINEAR);

     // ✅ BEV 이미지 저장 (추가)
    {
        std::lock_guard<std::mutex> lk(bev_mutex_);
        bev_bgr_ = bev.clone();   // 멤버 변수에 저장
    }

    // 표시
    cv::Mat bev_rgb;
    cv::cvtColor(bev, bev_rgb, cv::COLOR_BGR2RGB);
    QImage qimg_bev((const uchar*)bev_rgb.data, bev_rgb.cols, bev_rgb.rows, static_cast<int>(bev_rgb.step), QImage::Format_RGB888);
    outLabel->setPixmap(QPixmap::fromImage(qimg_bev.copy()).scaled(outLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));

    // 원본 영상에 srcQuad 윤곽을 그려 표시하고 싶다면 drawBorderOnSrc=true로 drawLabel에 그릴 수 있음
    if (drawBorderOnSrc && moveUi && moveUi->Bounding_Box) {
        QPixmap pm = moveUi->Bounding_Box->pixmap(Qt::ReturnByValue);
        if (!pm.isNull()) {
            QPixmap pix = pm.copy();
            int orig_w = orig.cols, orig_h = orig.rows;
            double sx = double(pix.width())  / double(orig_w);
            double sy = double(pix.height()) / double(orig_h);

            QPainter painter(&pix);
            QPen pen(borderColor);
            pen.setWidth(2);
            painter.setPen(pen);
            // 매핑 후 그리기
            QPolygon poly;
            for (int i=0;i<4;++i) {
                int px = std::max(0, std::min(pix.width()-1, static_cast<int>(std::round(srcQuad[i].x * sx))));
                int py = std::max(0, std::min(pix.height()-1, static_cast<int>(std::round(srcQuad[i].y * sy))));
                poly << QPoint(px, py);
            }
            painter.drawPolygon(poly);
            painter.end();
            moveUi->Bounding_Box->setPixmap(pix);
        }
    }
}

void MainWindow::updateMask(cv::Mat &mask, const cv::Scalar &lower, const cv::Scalar &upper, QLabel *label)
{
    cv::Mat hsv_copy;

    if ((label == moveUi->white_line) || (label == moveUi->yellow_line) || (label == moveUi->red_line)) {
        // 버드아이즈 이미지 사용
        std::lock_guard<std::mutex> lk(bev_mutex_);
        if (bev_bgr_.empty()) return;
        cv::cvtColor(bev_bgr_, hsv_copy, cv::COLOR_BGR2HSV);
    } else {
        // 일반 HSV 이미지 사용
        std::lock_guard<std::mutex> lk(img_mutex_);
        if (img_hsv_.empty()) return;
        hsv_copy = img_hsv_.clone();
    }

    // 일반적인 색상 처리
    if (label != moveUi->red_line) {
        cv::inRange(hsv_copy, lower, upper, mask);
    } 
    // 빨간선은 두 범위 합치기
    else {
        cv::Mat mask1, mask2;
        cv::inRange(hsv_copy, lower_red_line_low, upper_red_line_low, mask1);
        cv::inRange(hsv_copy, lower_red_line_high, upper_red_line_high, mask2);
        cv::bitwise_or(mask1, mask2, mask);
    }

    // 빨간선 제거
    if (!red_line_mask_.empty() && 
        (label == moveUi->white_line || label == moveUi->yellow_line)) {
        cv::Mat not_red, red_bin;
        cv::threshold(red_line_mask_, red_bin, 127, 255, cv::THRESH_BINARY);
        cv::bitwise_not(red_bin, not_red);
        cv::bitwise_and(mask, not_red, mask);
    }

    // 전처리
    cv::GaussianBlur(mask, mask, cv::Size(5,5), 0);
    cv::erode(mask, mask, cv::Mat(), cv::Point(-1,-1), 1);
    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1,-1), 1);

    cv::morphologyEx(mask, mask, cv::MORPH_OPEN,
                 cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7,7)));

    // QLabel 표시
    QImage qimg(mask.data, mask.cols, mask.rows, static_cast<int>(mask.step), QImage::Format_Grayscale8);
    label->setPixmap(QPixmap::fromImage(qimg.copy()).scaled(label->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));

    // --- 마스크 저장 ---
    if (label == moveUi->white_line) {
        white_line_mask_ = mask.clone();
    } else if (label == moveUi->yellow_line) {
        // 내부 구멍 채우기
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE,
                        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(10,10)));
        yellow_line_mask_ = mask.clone();
    } else if (label == moveUi->red_line) {
        red_line_mask_ = mask.clone();
    }

    // ---- 화이트 + 옐로우 둘 다 있으면 fine_line 계산 ----
    if (!white_line_mask_.empty() && !yellow_line_mask_.empty()) {

        // // red_line_mask_가 존재한다면 그 부분 제거
        // if (!red_line_mask_.empty()) {
        //     cv::Mat not_red;
        //     cv::bitwise_not(red_line_mask_, not_red);

        //     cv::bitwise_and(white_line_mask_, not_red, white_line_mask_);
        //     cv::bitwise_and(yellow_line_mask_, not_red, yellow_line_mask_);
        // }

        // 차선 검출 실행
        findLaneAndPublish(yellow_line_mask_, white_line_mask_);
    }
}

// QLabel에 이미지 표시
void MainWindow::updateImage(const QImage &img)
{
    if (!moveUi || !moveUi->video || img.isNull()) return;

    // 1) 기준 원본 이미지를 안전하게 가져오기
    cv::Mat orig;
    {
        std::lock_guard<std::mutex> lk(img_mutex_);
        if (!img_bgr_.empty()) orig = img_bgr_.clone();
    }

    QPixmap basePixmap;
    if (!orig.empty()) {
        cv::Mat rgb;
        cv::cvtColor(orig, rgb, cv::COLOR_BGR2RGB);
        QImage qorig(reinterpret_cast<const uchar*>(rgb.data),
                     rgb.cols, rgb.rows, static_cast<int>(rgb.step),
                     QImage::Format_RGB888);
        basePixmap = QPixmap::fromImage(qorig.copy()); // 깊은 복사
    } else {
        // fallback: img 파라미터 사용 (이미 안전한 복사로 전송됨)
        basePixmap = QPixmap::fromImage(img.copy());
    }

    // 2) 모든 라벨에 동일한 base에서 파생된 pixmap을 먼저 넣어둠
    moveUi->video->setScaledContents(false);
    moveUi->video->setPixmap(basePixmap.scaled(moveUi->video->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));

    if (moveUi->Bounding_Box) {
        moveUi->Bounding_Box->setScaledContents(false);
        moveUi->Bounding_Box->setPixmap(basePixmap.scaled(moveUi->Bounding_Box->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    }
    // (birds-eye도 base에서 파생해서 사용하면 간섭 감소)
    if (moveUi->yellow_con) {
        moveUi->yellow_con->setScaledContents(false);
        moveUi->yellow_con->setPixmap(basePixmap.scaled(moveUi->yellow_con->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    }

    // 라벨 크기 리포트 (한 번만)
    static bool label_size_reported = false;
    if (!label_size_reported) {
        int label_w = moveUi->video->width();
        int label_h = moveUi->video->height();
        RCLCPP_INFO(ros_node_->get_logger(), "Label size: %d x %d, Image size: %d x %d",
                    label_w, label_h, img.width(), img.height());
        statusBar()->showMessage(QString("Label: %1x%2  Image: %3x%4")
                                .arg(label_w).arg(label_h).arg(img.width()).arg(img.height()), 5000);
        label_size_reported = true;
    }

    auto quad = computeBirdsEyeQuad();
    showBirdsEye(quad, cv::Size(400,300), moveUi->yellow_con, true, Qt::green);

    // 3) 이후 처리: 마스크/박스/수평선 등 (processAndDrawBBox는 drawLabel의 pixmap을 복사해서 사용함)
    if (moveUi) {
        cv::Mat mask;
        // 마스크 및 라벨 표시(작업 순서 일관)
        //updateMask(mask, lower_white_line, upper_white_line, moveUi->yellow_con);
        updateMask(mask, lower_red_line_low, upper_red_line_low, moveUi->red_line);
        updateMask(mask, lower_white_line, upper_white_line, moveUi->white_line);
        updateMask(mask, lower_yellow_line, upper_yellow_line, moveUi->yellow_line);
        
        updateMask(mask, lower_traffic, upper_traffic, moveUi->traffic);
        updateMask(mask, lower_block, upper_block, moveUi->block);
        updateMask(mask, lower_gate, upper_gate, moveUi->gate);

        // processAndDrawBBox는 drawLabel에 설정된 pixmap을 복사해서 그리므로
        processAndDrawBBox(lower_white_line, upper_white_line, Qt::white,
                           moveUi->white_line, moveUi->white_line, "white_line", roi_white_line, bbox_white_line);
        processAndDrawBBox(lower_yellow_line, upper_yellow_line, QColor(255,200,0),
                           moveUi->yellow_line, moveUi->yellow_line, "yellow_line", roi_yellow_line, bbox_yellow_line);

        processAndDrawBBox(lower_traffic, upper_traffic, Qt::magenta,
                            moveUi->traffic, moveUi->Bounding_Box, "traffic", roi_traffic, bbox_traffic);
        processAndDrawBBox(lower_red_line_low, upper_red_line_low, Qt::red,
                            moveUi->red_line, moveUi->red_line, "red_line", roi_red_line, bbox_red_line);
        drawHorizontalLineOnLabel(moveUi->red_line, red_y, Qt::red, 2);
        
        processAndDrawBBox(lower_block, upper_block, Qt::gray,
                            moveUi->block, moveUi->Bounding_Box, "block", roi_block, bbox_block);
        processAndDrawBBox(lower_gate, upper_gate, Qt::red,
                            moveUi->gate, moveUi->gate, "gate", roi_gate, bbox_gate);
        drawHorizontalLineOnLabel(moveUi->gate, gate_y, Qt::red, 2);

        drawHorizontalLineOnLabel(moveUi->Bounding_Box, line_y, Qt::red, 2);
    }

    // 게이트 검출 플래그 설정
    if (std::abs((bbox_gate[1] + bbox_gate[3] / 2) - gate_y) <= 10) {
        if ((bbox_gate[2] * bbox_gate[3]) > 30000) {
            gate_detected = true;
        } else {
            gate_detected = false;
        }
    } else {
        gate_detected = false;
    }

    // 빨간선 검출 플래그 설정
    if (std::abs((bbox_red_line[1] + bbox_red_line[3] / 2) - (280 + (red_y * (410 - 280)) / 480)) <= 10) {
        if ((bbox_red_line[2] * bbox_red_line[3]) > 18000) {
            red_flag = true;
        } else {
            red_flag = false;
        }
        red_flag = true;
    } else {
        red_flag = false;
    }

    // // 4) birds-eye는 별도 함수에서 orig 기반으로 생성하므로 간섭 없음
    // std::array<cv::Point2f,4> quad = { cv::Point2f(100,200), cv::Point2f(540,200), cv::Point2f(540,400), cv::Point2f(100,400) };
    // showBirdsEye(quad, cv::Size(400,300), moveUi->yellow_con, true, Qt::green);

}

void MainWindow::processAndDrawBBox(const cv::Scalar &lower, const cv::Scalar &upper,
                                    const QColor &penColor, QLabel *maskLabel, QLabel *drawLabel,
                                    const QString &logName, const std::array<int,4> &roi, std::array<int,4> &out_bbox)
{
    // 기본 none 표시
    out_bbox = std::array<int,4>{-1,-1,0,0};

    // 원본 HSV 복사 (스레드 안전)
    cv::Mat hsv;
    cv::Mat mask;

    if (logName == "red_line") {
        // 버드아이 이미지 기반의 레드라인은 updateMask()에서 이미 만들어둔 걸 그대로 사용
        std::lock_guard<std::mutex> lk(bev_mutex_);
        if (red_line_mask_.empty()) return;
        mask = red_line_mask_.clone();
    }
    else {
        // 일반 HSV 이미지 기반
        std::lock_guard<std::mutex> lk(img_mutex_);
        if (img_hsv_.empty()) return;
        hsv = img_hsv_.clone();

        // 전체 마스크 생성 + 전처리
        cv::inRange(hsv, lower, upper, mask);
        cv::GaussianBlur(mask, mask, cv::Size(5,5), 0);
        cv::erode(mask, mask, cv::Mat(), cv::Point(-1,-1), 1);
        cv::dilate(mask, mask, cv::Mat(), cv::Point(-1,-1), 1);

        cv::morphologyEx(mask, mask, cv::MORPH_OPEN,
                 cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7,7)));
        if (logName == "yellow_line") {
            // 내부 구멍 채우기
            cv::morphologyEx(mask, mask, cv::MORPH_CLOSE,
                            cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7,7)));
        }
    }

    // maskLabel에 전체 마스크 또는 ROI 마스크 표시
    if (maskLabel) {
        QImage qimg_mask(
            mask.data,
            mask.cols,
            mask.rows,
            static_cast<int>(mask.step),
            QImage::Format_Grayscale8
        );

        maskLabel->setPixmap(
            QPixmap::fromImage(qimg_mask.copy())
                .scaled(maskLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation)
        );
    }

    // ROI 설정 (roi = {x, y, w, h}); w,h <=0 이면 전체 사용
    int rx = roi[0], ry = roi[1], rw = roi[2], rh = roi[3];
    cv::Rect roiRect;
    if (rw > 0 && rh > 0) {
        // clip
        rx = std::max(0, std::min(rx, mask.cols-1));
        ry = std::max(0, std::min(ry, mask.rows-1));
        rw = std::max(1, std::min(rw, mask.cols - rx));
        rh = std::max(1, std::min(rh, mask.rows - ry));
        roiRect = cv::Rect(rx, ry, rw, rh);
    } else {
        roiRect = cv::Rect(0, 0, mask.cols, mask.rows);
    }

    // 컨투어 검출 (ROI 기준)
    cv::Mat mask_roi = mask(roiRect);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask_roi, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) {
        //if (!logName.isEmpty()) moveUi->textEdit->append(logName + ": 없음");
        return;
    }

    // 면적 필터링 후 포인트 통합
    std::vector<cv::Point> pts;
    for (const auto &c : contours) {
        if (cv::contourArea(c) > 600.0) {
            // contour 좌표는 ROI 기준이므로 그대로 추가
            pts.insert(pts.end(), c.begin(), c.end());
        }
    }
    if (pts.empty()) {
        //if (!logName.isEmpty()) moveUi->textEdit->append(logName + ": 없음");
        return;
    }

    // ROI 기준 bbox, 그 후 전역 좌표로 변환
    cv::Rect bbox_local = cv::boundingRect(pts);
    cv::Rect bbox_global(bbox_local.x + roiRect.x, bbox_local.y + roiRect.y, bbox_local.width, bbox_local.height);

    // 레드 라인일 경우 전역 좌표로 변환하지 말고 ROI 기준으로 그대로 사용
    if (logName == "red_line") {
        bbox_global = cv::Rect(bbox_local.x, bbox_local.y, bbox_local.width, bbox_local.height);
    }

    // 결과 저장
    out_bbox = std::array<int,4>{ bbox_global.x, bbox_global.y, bbox_global.width, bbox_global.height };

    // drawLabel의 pixmap을 가져와서 원본->픽스맵으로 스케일 매핑 후 그리기
    if (!drawLabel) return;
    QPixmap pm = drawLabel->pixmap(Qt::ReturnByValue);
    if (pm.isNull()) return;
    QPixmap pix = pm.copy();

    // 원본 크기(HSV 멤버)
    int orig_w = 0, orig_h = 0;
    {
        std::lock_guard<std::mutex> lk(img_mutex_);
        orig_h = img_hsv_.rows;
        orig_w = img_hsv_.cols;
    }
    if (orig_w <= 0 || orig_h <= 0) { orig_w = pix.width(); orig_h = pix.height(); }

    // 레드 라인인 경우 ROI 기준으로 그리기: orig_w/orig_h를 ROI 크기로 바꿔서 매핑
    if (logName == "red_line") {
        if (roiRect.width > 0 && roiRect.height > 0) {
            orig_w = roiRect.width;
            orig_h = roiRect.height;
        }
    }

    double sx = double(pix.width())  / double(orig_w);
    double sy = double(pix.height()) / double(orig_h);

    int x, y, w, h;
    if (logName == "red_line") {
        // bbox_global은 ROI 기준(bbox_local)으로 설정되어 있으므로 ROI 기준으로 스케일
        x = std::max(0, std::min(pix.width()-1, static_cast<int>(std::round(bbox_global.x * sx))));
        y = std::max(0, std::min(pix.height()-1, static_cast<int>(std::round(bbox_global.y * sy))));
        w = std::max(1, std::min(pix.width()-x, static_cast<int>(std::round(bbox_global.width * sx))));
        h = std::max(1, std::min(pix.height()-y, static_cast<int>(std::round(bbox_global.height * sy))));
    } else {
        // 일반: 전체 이미지 기준으로 ROI 오프셋을 포함한 전역 bbox_global을 사용
        x = std::max(0, std::min(pix.width()-1, static_cast<int>(std::round(bbox_global.x * sx))));
        y = std::max(0, std::min(pix.height()-1, static_cast<int>(std::round(bbox_global.y * sy))));
        w = std::max(1, std::min(pix.width()-x, static_cast<int>(std::round(bbox_global.width * sx))));
        h = std::max(1, std::min(pix.height()-y, static_cast<int>(std::round(bbox_global.height * sy))));
    }

    QPainter painter(&pix);
    QPen pen(penColor);
    pen.setWidth(2);
    painter.setPen(pen);
    painter.drawRect(x, y, w, h);
    painter.end();

    drawLabel->setPixmap(pix);
}

void MainWindow::onMoveHLowChanged(int value) {
    if (moveUi->white_line_2->isChecked())   lower_white_line[0] = value;
    if (moveUi->yellow_line_2->isChecked())  lower_yellow_line[0] = value;
    if (moveUi->red_line_low->isChecked())     lower_red_line_low[0] = value;
    if (moveUi->red_line_high->isChecked())     lower_red_line_high[0] = value;
    if (moveUi->block_2->isChecked())        lower_block[0] = value;
    if (moveUi->gate_2->isChecked())         lower_gate[0] = value;
    if (moveUi->traffic_2->isChecked())      lower_traffic[0] = value;

    if (moveUi->traffic_red->isChecked())      lower_traffic_red[0] = value;
    if (moveUi->traffic_yellow->isChecked())      lower_traffic_yellow[0] = value;
    if (moveUi->traffic_green->isChecked())      lower_traffic_green[0] = value;
    
    moveUi->H_low_index->setText(QString::number(value));
}

void MainWindow::onMoveHHighChanged(int value) {
    if (moveUi->white_line_2->isChecked())   upper_white_line[0] = value;
    if (moveUi->yellow_line_2->isChecked())  upper_yellow_line[0] = value;
    if (moveUi->red_line_low->isChecked())     upper_red_line_low[0] = value;
    if (moveUi->red_line_high->isChecked())     upper_red_line_high[0] = value;
    if (moveUi->block_2->isChecked())        upper_block[0] = value;
    if (moveUi->gate_2->isChecked())         upper_gate[0] = value;
    if (moveUi->traffic_2->isChecked())      upper_traffic[0] = value;

    if (moveUi->traffic_red->isChecked())      upper_traffic_red[0] = value;
    if (moveUi->traffic_yellow->isChecked())      upper_traffic_yellow[0] = value;
    if (moveUi->traffic_green->isChecked())      upper_traffic_green[0] = value;

    moveUi->H_high_index->setText(QString::number(value));
}

void MainWindow::onMoveSLowChanged(int value) {
    if (moveUi->white_line_2->isChecked())   lower_white_line[1] = value;
    if (moveUi->yellow_line_2->isChecked())  lower_yellow_line[1] = value;
    if (moveUi->red_line_low->isChecked())     lower_red_line_low[1] = value;
    if (moveUi->red_line_high->isChecked())     lower_red_line_high[1] = value;
    if (moveUi->block_2->isChecked())        lower_block[1] = value;
    if (moveUi->gate_2->isChecked())         lower_gate[1] = value;
    if (moveUi->traffic_2->isChecked())      lower_traffic[1] = value;

    if (moveUi->traffic_red->isChecked())      lower_traffic_red[1] = value;
    if (moveUi->traffic_yellow->isChecked())      lower_traffic_yellow[1] = value;
    if (moveUi->traffic_green->isChecked())      lower_traffic_green[1] = value;

    moveUi->S_low_index->setText(QString::number(value));
}

void MainWindow::onMoveSHighChanged(int value) {
    if (moveUi->white_line_2->isChecked())   upper_white_line[1] = value;
    if (moveUi->yellow_line_2->isChecked())  upper_yellow_line[1] = value;
    if (moveUi->red_line_low->isChecked())     upper_red_line_low[1] = value;
    if (moveUi->red_line_high->isChecked())     upper_red_line_high[1] = value;
    if (moveUi->block_2->isChecked())        upper_block[1] = value;
    if (moveUi->gate_2->isChecked())         upper_gate[1] = value;
    if (moveUi->traffic_2->isChecked())      upper_traffic[1] = value;

    if (moveUi->traffic_red->isChecked())      upper_traffic_red[1] = value;
    if (moveUi->traffic_yellow->isChecked())      upper_traffic_yellow[1] = value;
    if (moveUi->traffic_green->isChecked())      upper_traffic_green[1] = value;

    moveUi->S_high_index->setText(QString::number(value));
}

void MainWindow::onMoveKLowChanged(int value) {
    if (moveUi->white_line_2->isChecked())   lower_white_line[2] = value;
    if (moveUi->yellow_line_2->isChecked())  lower_yellow_line[2] = value;
    if (moveUi->red_line_low->isChecked())     lower_red_line_low[2] = value;
    if (moveUi->red_line_high->isChecked())     lower_red_line_high[2] = value;
    if (moveUi->block_2->isChecked())        lower_block[2] = value;
    if (moveUi->gate_2->isChecked())         lower_gate[2] = value;
    if (moveUi->traffic_2->isChecked())      lower_traffic[2] = value;

    if (moveUi->traffic_red->isChecked())      lower_traffic_red[2] = value;
    if (moveUi->traffic_yellow->isChecked())      lower_traffic_yellow[2] = value;
    if (moveUi->traffic_green->isChecked())      lower_traffic_green[2] = value;

    moveUi->K_low_index->setText(QString::number(value));
}

void MainWindow::onMoveKHighChanged(int value) {
    if (moveUi->white_line_2->isChecked())   upper_white_line[2] = value;
    if (moveUi->yellow_line_2->isChecked())  upper_yellow_line[2] = value;
    if (moveUi->red_line_low->isChecked())     upper_red_line_low[2] = value;
    if (moveUi->red_line_high->isChecked())     upper_red_line_high[2] = value;
    if (moveUi->block_2->isChecked())        upper_block[2] = value;
    if (moveUi->gate_2->isChecked())         upper_gate[2] = value;
    if (moveUi->traffic_2->isChecked())      upper_traffic[2] = value;

    if (moveUi->traffic_red->isChecked())      upper_traffic_red[2] = value;
    if (moveUi->traffic_yellow->isChecked())      upper_traffic_yellow[2] = value;
    if (moveUi->traffic_green->isChecked())      upper_traffic_green[2] = value;

    moveUi->K_high_index->setText(QString::number(value));
}

void MainWindow::drawHorizontalLineOnLabel(QLabel *label, int y_value, const QColor &color, int thickness)
{
    if (!label) return;
    QPixmap pm = label->pixmap(Qt::ReturnByValue);
    if (pm.isNull()) return;

    QPixmap pix = pm.copy(); // QLabel에 표시된 현재 픽스맵 복사

    if (pix.isNull()) return;

    // 원본 이미지 높이(멤버 img_hsv_ 사용, 스레드 안전)
    int orig_h = 0;
    {
        std::lock_guard<std::mutex> lk(img_mutex_);
        orig_h = img_hsv_.rows;
    }
    if (orig_h <= 0) {
        // 원본 정보가 없으면 pixmap 높이를 원본으로 간주
        orig_h = pix.height();
    }

    // 원본 y 값을 pixmap 좌표계로 매핑
    double scale = double(pix.height()) / double(orig_h);
    int y = static_cast<int>(std::round(y_value * scale));
    y = std::max(0, std::min(y, pix.height() - 1));

    QPainter painter(&pix);
    painter.setRenderHint(QPainter::Antialiasing, false);
    QPen pen(color);
    pen.setWidth(thickness);
    painter.setPen(pen);
    painter.drawLine(0, y, pix.width() - 1, y);
    painter.end();

    label->setPixmap(pix);
}

// 가로선
void MainWindow::onMoveRedChanged(int value) {
    red_y = value;
    //moveUi->K_high_index->setText(QString::number(value));
}

void MainWindow::onMoveGateChanged(int value) {
    gate_y = value;
    //moveUi->K_high_index->setText(QString::number(value));
}

void MainWindow::onMoveLineChanged(int value) {
    line_y = value;
    //moveUi->K_high_index->setText(QString::number(value));
}

// 버드아이즈
void MainWindow::onBXH_Changed(int value) {
    birdseye_x_high = value;
    moveUi->B_x_high->setText(QString::number(value));

    auto quad = computeBirdsEyeQuad();
    showBirdsEye(quad, cv::Size(400,300), moveUi->yellow_con, true, Qt::green);
}

void MainWindow::onBXL_Changed(int value) {
    birdseye_x_low = value;
    moveUi->B_x_low->setText(QString::number(value));

    auto quad = computeBirdsEyeQuad();
    showBirdsEye(quad, cv::Size(400,300), moveUi->yellow_con, true, Qt::green);
}

void MainWindow::onBYH_Changed(int value) {
    birdseye_y_high = value;
    moveUi->B_y_high->setText(QString::number(value));

    auto quad = computeBirdsEyeQuad();
    showBirdsEye(quad, cv::Size(400,300), moveUi->yellow_con, true, Qt::green);
}

void MainWindow::onBYL_Changed(int value) {
    birdseye_y_low = value;
    moveUi->B_y_low->setText(QString::number(value));

    auto quad = computeBirdsEyeQuad();
    showBirdsEye(quad, cv::Size(400,300), moveUi->yellow_con, true, Qt::green);
}


void MainWindow::onMoveWhiteLineCheckChanged(bool checked) {
    if (checked) {
        moveUi->yellow_line_2->setChecked(false);
        moveUi->red_line_low->setChecked(false);
        moveUi->red_line_high->setChecked(false);
        moveUi->block_2->setChecked(false);
        moveUi->gate_2->setChecked(false);
        moveUi->traffic_2->setChecked(false);
        moveUi->traffic_red->setChecked(false);
        moveUi->traffic_yellow->setChecked(false);
        moveUi->traffic_green->setChecked(false);

        moveUi->H_low_slider->setValue(lower_white_line[0]);
        moveUi->H_high_slider->setValue(upper_white_line[0]);
        moveUi->S_low_slider->setValue(lower_white_line[1]);
        moveUi->S_high_slider->setValue(upper_white_line[1]);
        moveUi->K_low_slider->setValue(lower_white_line[2]);
        moveUi->K_high_slider->setValue(upper_white_line[2]);
    }
}

void MainWindow::onMoveYellowLineCheckChanged(bool checked) {
    if (checked) {
        moveUi->white_line_2->setChecked(false);
        moveUi->red_line_low->setChecked(false);
        moveUi->red_line_high->setChecked(false);
        moveUi->block_2->setChecked(false);
        moveUi->gate_2->setChecked(false);
        moveUi->traffic_2->setChecked(false);
        moveUi->traffic_red->setChecked(false);
        moveUi->traffic_yellow->setChecked(false);
        moveUi->traffic_green->setChecked(false);

        moveUi->H_low_slider->setValue(lower_yellow_line[0]);
        moveUi->H_high_slider->setValue(upper_yellow_line[0]);
        moveUi->S_low_slider->setValue(lower_yellow_line[1]);
        moveUi->S_high_slider->setValue(upper_yellow_line[1]);
        moveUi->K_low_slider->setValue(lower_yellow_line[2]);
        moveUi->K_high_slider->setValue(upper_yellow_line[2]);
    }
}

void MainWindow::onMoveRedLineLCheckChanged(bool checked) {
    if (checked) {
        moveUi->white_line_2->setChecked(false);
        moveUi->yellow_line_2->setChecked(false);
        moveUi->red_line_high->setChecked(false);
        moveUi->block_2->setChecked(false);
        moveUi->gate_2->setChecked(false);
        moveUi->traffic_2->setChecked(false);
        moveUi->traffic_red->setChecked(false);
        moveUi->traffic_yellow->setChecked(false);
        moveUi->traffic_green->setChecked(false);

        moveUi->H_low_slider->setValue(lower_red_line_low[0]);
        moveUi->H_high_slider->setValue(upper_red_line_low[0]);
        moveUi->S_low_slider->setValue(lower_red_line_low[1]);
        moveUi->S_high_slider->setValue(upper_red_line_low[1]);
        moveUi->K_low_slider->setValue(lower_red_line_low[2]);
        moveUi->K_high_slider->setValue(upper_red_line_low[2]);
    }
}

void MainWindow::onMoveRedLineHCheckChanged(bool checked) {
    if (checked) {
        moveUi->white_line_2->setChecked(false);
        moveUi->yellow_line_2->setChecked(false);
        moveUi->red_line_low->setChecked(false);
        moveUi->block_2->setChecked(false);
        moveUi->gate_2->setChecked(false);
        moveUi->traffic_2->setChecked(false);
        moveUi->traffic_red->setChecked(false);
        moveUi->traffic_yellow->setChecked(false);
        moveUi->traffic_green->setChecked(false);

        moveUi->H_low_slider->setValue(lower_red_line_high[0]);
        moveUi->H_high_slider->setValue(upper_red_line_high[0]);
        moveUi->S_low_slider->setValue(lower_red_line_high[1]);
        moveUi->S_high_slider->setValue(upper_red_line_high[1]);
        moveUi->K_low_slider->setValue(lower_red_line_high[2]);
        moveUi->K_high_slider->setValue(upper_red_line_high[2]);
    }
}

void MainWindow::onMoveBlockCheckChanged(bool checked) {
    if (checked) {
        moveUi->white_line_2->setChecked(false);
        moveUi->yellow_line_2->setChecked(false);
        moveUi->red_line_low->setChecked(false);
        moveUi->red_line_high->setChecked(false);
        moveUi->gate_2->setChecked(false);
        moveUi->traffic_2->setChecked(false);
        moveUi->traffic_red->setChecked(false);
        moveUi->traffic_yellow->setChecked(false);
        moveUi->traffic_green->setChecked(false);

        moveUi->H_low_slider->setValue(lower_block[0]);
        moveUi->H_high_slider->setValue(upper_block[0]);
        moveUi->S_low_slider->setValue(lower_block[1]);
        moveUi->S_high_slider->setValue(upper_block[1]);
        moveUi->K_low_slider->setValue(lower_block[2]);
        moveUi->K_high_slider->setValue(upper_block[2]);
    }
}

void MainWindow::onMoveGateCheckChanged(bool checked) {
    if (checked) {
        moveUi->white_line_2->setChecked(false);
        moveUi->yellow_line_2->setChecked(false);
        moveUi->red_line_low->setChecked(false);
        moveUi->red_line_high->setChecked(false);
        moveUi->block_2->setChecked(false);
        moveUi->traffic_2->setChecked(false);
        moveUi->traffic_red->setChecked(false);
        moveUi->traffic_yellow->setChecked(false);
        moveUi->traffic_green->setChecked(false);

        moveUi->H_low_slider->setValue(lower_gate[0]);
        moveUi->H_high_slider->setValue(upper_gate[0]);
        moveUi->S_low_slider->setValue(lower_gate[1]);
        moveUi->S_high_slider->setValue(upper_gate[1]);
        moveUi->K_low_slider->setValue(lower_gate[2]);
        moveUi->K_high_slider->setValue(upper_gate[2]);
    }
}

void MainWindow::onMoveTrafficCheckChanged(bool checked) {
    if (checked) {
        moveUi->white_line_2->setChecked(false);
        moveUi->yellow_line_2->setChecked(false);
        moveUi->red_line_low->setChecked(false);
        moveUi->red_line_high->setChecked(false);
        moveUi->block_2->setChecked(false);
        moveUi->gate_2->setChecked(false);
        moveUi->traffic_red->setChecked(false);
        moveUi->traffic_yellow->setChecked(false);
        moveUi->traffic_green->setChecked(false);

        moveUi->H_low_slider->setValue(lower_traffic[0]);
        moveUi->H_high_slider->setValue(upper_traffic[0]);
        moveUi->S_low_slider->setValue(lower_traffic[1]);
        moveUi->S_high_slider->setValue(upper_traffic[1]);
        moveUi->K_low_slider->setValue(lower_traffic[2]);
        moveUi->K_high_slider->setValue(upper_traffic[2]);
    }
}

void MainWindow::onMoveTrafficRedCheckChanged(bool checked) {
    if (checked) {
        moveUi->white_line_2->setChecked(false);
        moveUi->yellow_line_2->setChecked(false);
        moveUi->red_line_low->setChecked(false);
        moveUi->red_line_high->setChecked(false);
        moveUi->block_2->setChecked(false);
        moveUi->gate_2->setChecked(false);
        moveUi->traffic_2->setChecked(false);
        moveUi->traffic_yellow->setChecked(false);
        moveUi->traffic_green->setChecked(false);

        moveUi->H_low_slider->setValue(lower_traffic_red[0]);
        moveUi->H_high_slider->setValue(upper_traffic_red[0]);
        moveUi->S_low_slider->setValue(lower_traffic_red[1]);
        moveUi->S_high_slider->setValue(upper_traffic_red[1]);
        moveUi->K_low_slider->setValue(lower_traffic_red[2]);
        moveUi->K_high_slider->setValue(upper_traffic_red[2]);
    }
}

void MainWindow::onMoveTrafficYellowCheckChanged(bool checked) {
    if (checked) {
        moveUi->white_line_2->setChecked(false);
        moveUi->yellow_line_2->setChecked(false);
        moveUi->red_line_low->setChecked(false);
        moveUi->red_line_high->setChecked(false);
        moveUi->block_2->setChecked(false);
        moveUi->gate_2->setChecked(false);
        moveUi->traffic_2->setChecked(false);
        moveUi->traffic_red->setChecked(false);
        moveUi->traffic_green->setChecked(false);

        moveUi->H_low_slider->setValue(lower_traffic_yellow[0]);
        moveUi->H_high_slider->setValue(upper_traffic_yellow[0]);
        moveUi->S_low_slider->setValue(lower_traffic_yellow[1]);
        moveUi->S_high_slider->setValue(upper_traffic_yellow[1]);
        moveUi->K_low_slider->setValue(lower_traffic_yellow[2]);
        moveUi->K_high_slider->setValue(upper_traffic_yellow[2]);
    }
}

void MainWindow::onMoveTrafficGreenCheckChanged(bool checked) {
    if (checked) {
        moveUi->white_line_2->setChecked(false);
        moveUi->yellow_line_2->setChecked(false);
        moveUi->red_line_low->setChecked(false);
        moveUi->red_line_high->setChecked(false);
        moveUi->block_2->setChecked(false);
        moveUi->gate_2->setChecked(false);
        moveUi->traffic_2->setChecked(false);
        moveUi->traffic_red->setChecked(false);
        moveUi->traffic_yellow->setChecked(false);

        moveUi->H_low_slider->setValue(lower_traffic_green[0]);
        moveUi->H_high_slider->setValue(upper_traffic_green[0]);
        moveUi->S_low_slider->setValue(lower_traffic_green[1]);
        moveUi->S_high_slider->setValue(upper_traffic_green[1]);
        moveUi->K_low_slider->setValue(lower_traffic_green[2]);
        moveUi->K_high_slider->setValue(upper_traffic_green[2]);
    }
}

void MainWindow::onMoveStartClicked()
{
    is_started_ = 1;
    moveUi->start->setStyleSheet("background-color: green;");
    moveUi->stop->setStyleSheet("");
    RCLCPP_INFO(ros_node_->get_logger(), "Move Start pressed");
}

void MainWindow::onMoveStopClicked()
{
    is_started_ = 0;
    moveUi->stop->setStyleSheet("background-color: red;");
    moveUi->start->setStyleSheet("");
    RCLCPP_INFO(ros_node_->get_logger(), "Move Stop pressed");
}

void MainWindow::onHSVResetClicked()
{
    // HSV 초기값
    lower_white_line  = cv::Scalar(0, 0, 133);
    upper_white_line  = cv::Scalar(157, 29, 255);
    lower_yellow_line = cv::Scalar(0, 40, 111);
    upper_yellow_line = cv::Scalar(90, 255, 255);
    lower_red_line_low    = cv::Scalar(0, 60, 0);
    upper_red_line_low    = cv::Scalar(17, 255, 255);
    lower_red_line_high   = cv::Scalar(160, 60, 0);
    upper_red_line_high   = cv::Scalar(179, 255, 255);
    lower_block       = cv::Scalar(100, 100, 100);
    upper_block       = cv::Scalar(130, 255, 255);
    lower_gate        = cv::Scalar(40, 50, 50);
    upper_gate        = cv::Scalar(90, 255, 255);
    lower_traffic     = cv::Scalar(0, 50, 50);
    upper_traffic     = cv::Scalar(10, 255, 255);
    lower_traffic_red       = cv::Scalar(0, 50, 50);
    upper_traffic_red       = cv::Scalar(10, 255, 255);
    lower_traffic_yellow    = cv::Scalar(0, 50, 50);
    upper_traffic_yellow    = cv::Scalar(10, 255, 255);
    lower_traffic_green     = cv::Scalar(0, 50, 50);
    upper_traffic_green     = cv::Scalar(10, 255, 255);

    // 상태 초기화
    use_white_line      = true;
    use_yellow_line     = false;
    use_red_line_L      = false;
    use_red_line_H      = false;
    use_block           = false;
    use_gate            = false;
    use_traffic         = false;
    use_sign            = false;
    use_traffic_red     = false;
    use_traffic_yellow  = false;
    use_traffic_green   = false;

    // ===== 체크박스 초기 상태 설정 =====
    moveUi->white_line_2->setChecked(true);
    moveUi->red_line_low->setChecked(false);
    moveUi->red_line_high->setChecked(false);
    moveUi->yellow_line_2->setChecked(false);
    moveUi->block_2->setChecked(false);
    moveUi->gate_2->setChecked(false);
    moveUi->traffic_2->setChecked(false);
    moveUi->traffic_red->setChecked(false);
    moveUi->traffic_yellow->setChecked(false);
    moveUi->traffic_green->setChecked(false);

    // ===== HSV 슬라이더 값 초기화 (White 기준) =====
    moveUi->H_low_slider->setValue(lower_white_line[0]);
    moveUi->H_high_slider->setValue(upper_white_line[0]);
    moveUi->S_low_slider->setValue(lower_white_line[1]);
    moveUi->S_high_slider->setValue(upper_white_line[1]);
    moveUi->K_low_slider->setValue(lower_white_line[2]);
    moveUi->K_high_slider->setValue(upper_white_line[2]);

    // ===== 표시 라벨 업데이트 =====
    moveUi->H_low_index->setText(QString::number(lower_white_line[0]));
    moveUi->H_high_index->setText(QString::number(upper_white_line[0]));
    moveUi->S_low_index->setText(QString::number(lower_white_line[1]));
    moveUi->S_high_index->setText(QString::number(upper_white_line[1]));
    moveUi->K_low_index->setText(QString::number(lower_white_line[2]));
    moveUi->K_high_index->setText(QString::number(upper_white_line[2]));
}
