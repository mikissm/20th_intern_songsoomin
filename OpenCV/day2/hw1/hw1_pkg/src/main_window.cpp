#include <QPixmap>
#include <QImage>
#include <opencv2/opencv.hpp>
#include "ui_mainwindow.h"
#include "../include/hw1_pkg/main_window.hpp"

using namespace cv;
using namespace std;

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // HSV 슬라이더 범위 설정
    ui->H_low_slider->setRange(0, 179);
    ui->H_high_slider->setRange(0, 179);
    ui->S_low_slider->setRange(0, 255);
    ui->S_high_slider->setRange(0, 255);
    ui->K_low_slider->setRange(0, 255);
    ui->K_high_slider->setRange(0, 255);

    // HSV 슬라이더 연결
    connect(ui->H_low_slider, &QSlider::valueChanged, this, &MainWindow::onHLowChanged);
    connect(ui->H_high_slider, &QSlider::valueChanged, this, &MainWindow::onHHighChanged);
    connect(ui->S_low_slider, &QSlider::valueChanged, this, &MainWindow::onSLowChanged);
    connect(ui->S_high_slider, &QSlider::valueChanged, this, &MainWindow::onSHighChanged);
    connect(ui->K_low_slider, &QSlider::valueChanged, this, &MainWindow::onKLowChanged);
    connect(ui->K_high_slider, &QSlider::valueChanged, this, &MainWindow::onKHighChanged);

    // 체크박스 연결
    connect(ui->white_line_2, &QCheckBox::toggled, this, &MainWindow::onWhiteLineCheckChanged);
    connect(ui->blue_line_2, &QCheckBox::toggled, this, &MainWindow::onBlueLineCheckChanged);
    connect(ui->yellow_con_2, &QCheckBox::toggled, this, &MainWindow::onYellowConCheckChanged);
    connect(ui->orange_con_2, &QCheckBox::toggled, this, &MainWindow::onOrangeConCheckChanged);

    // 버튼 연결
    connect(ui->pushButton, &QPushButton::clicked, this, &MainWindow::onTakePictureButtonClicked);
    connect(ui->hsv_reset, &QPushButton::clicked, this, &MainWindow::onHSVResetClicked);

    // 카메라 열기
    cap.open(2);
    if (!cap.isOpened()) {
        qDebug() << "카메라를 열 수 없습니다.";
    }

    // HSV 기본값
    lower_blue_line = Scalar(100, 100, 50);
    upper_blue_line = Scalar(130, 255, 255);

    lower_white_line = Scalar(0, 0, 245);
    upper_white_line = Scalar(180, 30, 255);

    lower_yellow_con = Scalar(26, 160, 100);
    upper_yellow_con = Scalar(30, 255, 255);

    lower_orange_con = Scalar(0, 100, 100);
    upper_orange_con = Scalar(20, 255, 255);

    // 이미지 갱신 타이머
    updateTimer = new QTimer(this);
    connect(updateTimer, &QTimer::timeout, this, &MainWindow::upDateImg);
    updateTimer->start(30);

    // 체크박스 초기화
    ui->white_line_2->setChecked(true);
    use_white_line = true;
    use_blue_line = false;
    use_yellow_con = false;
    use_orange_con = false;
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::closeEvent(QCloseEvent* event)
{
    QMainWindow::closeEvent(event);
}

// 이미지 업데이트
void MainWindow::upDateImg() {
    if (frame.empty()) return; // 찍힌 사진이 없으면 종료
    // HSV 변환
    cvtColor(frame, img_hsv, COLOR_BGR2HSV);

    // 마스크 업데이트 및 QLabel 표시
    updateMask(blue_line_mask, lower_blue_line, upper_blue_line, ui->blue_line);
    updateMask(white_line_mask, lower_white_line, upper_white_line, ui->white_line);
    updateMask(yellow_con_mask, lower_yellow_con, upper_yellow_con, ui->yellow_con);
    updateMask(orange_con_mask, lower_orange_con, upper_orange_con, ui->orange_con);

    // 찍힌 사진 기준 바운딩 박스 표시용 (모든 색)
    Mat displayFrame = frame.clone();
    drawBoundingBoxes(white_line_mask, displayFrame, "white");
    drawBoundingBoxes(blue_line_mask, displayFrame, "blue");
    drawBoundingBoxes(yellow_con_mask, displayFrame, "yellow");
    drawBoundingBoxes(orange_con_mask, displayFrame, "orange");


    // find_object QLabel에 표시
    Mat rgb;
    cvtColor(displayFrame, rgb, COLOR_BGR2RGB);
    QImage qimg((uchar*)rgb.data, rgb.cols, rgb.rows, rgb.step, QImage::Format_RGB888);
    ui->find_object->setPixmap(QPixmap::fromImage(qimg).scaled(ui->find_object->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));

    // HSV 값 QLabel 업데이트
    ui->H_low_index->setText(QString::number(ui->H_low_slider->value()));
    ui->H_high_index->setText(QString::number(ui->H_high_slider->value()));
    ui->S_low_index->setText(QString::number(ui->S_low_slider->value()));
    ui->S_high_index->setText(QString::number(ui->S_high_slider->value()));
    ui->K_low_index->setText(QString::number(ui->K_low_slider->value()));
    ui->K_high_index->setText(QString::number(ui->K_high_slider->value()));
}

// 마스크 업데이트
void MainWindow::updateMask(Mat &mask, const Scalar &lower, const Scalar &upper, QLabel *label)
{
    inRange(img_hsv, lower, upper, mask);

    // 전처리
    GaussianBlur(mask, mask, Size(5,5), 0);
    erode(mask, mask, Mat(), Point(-1,-1), 1);
    dilate(mask, mask, Mat(), Point(-1,-1), 1);

    // QLabel에 마스크 표시
    QImage qimg(mask.data, mask.cols, mask.rows, mask.step, QImage::Format_Grayscale8);
    label->setPixmap(QPixmap::fromImage(qimg).scaled(label->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
}

// 바운딩 박스 그리기 + 위치 추정 + 사진 잘라서 올리기
void MainWindow::drawBoundingBoxes(const cv::Mat &mask, cv::Mat &frame, const QString &colorName)
{
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) 
    {
        if (colorName == "yellow" || colorName == "orange") ui->textEdit->append(colorName + ": 없음");
        return;
    }

    std::vector<cv::Point> filteredPoints;
    for (const auto &c : contours) {
        if (cv::contourArea(c) > 600.0)
            filteredPoints.insert(filteredPoints.end(), c.begin(), c.end());
    }
    if (filteredPoints.empty())
    {
        if (colorName == "yellow" || colorName == "orange") ui->textEdit->append(colorName + ": 없음");
        return;
    }

    cv::Rect bbox = cv::boundingRect(filteredPoints);

    // 색상별 바운딩 박스 그리기
    cv::Scalar color;
    if(colorName == "blue") color = cv::Scalar(255,0,0);
    else if(colorName == "white") color = cv::Scalar(255,255,255);
    else if(colorName == "yellow") color = cv::Scalar(0,255,255);
    else if(colorName == "orange") color = cv::Scalar(0,165,255);
    cv::rectangle(frame, bbox, color, 2);

    // 색상별 바운더리 박스 저장
    if(colorName == "blue") bboxBlue = bbox;
    else if(colorName == "white") bboxWhite = bbox;
    else if(colorName == "yellow") bboxYellow = bbox;
    else if(colorName == "orange") bboxOrange = bbox;

    qDebug() << "bboxBlue:" << bboxBlue.x << bboxBlue.y << bboxBlue.width << bboxBlue.height;
    qDebug() << "bboxWhite:" << bboxWhite.x << bboxWhite.y << bboxWhite.width << bboxWhite.height;
    qDebug() << "bboxyellow:" << bboxYellow.x << bboxYellow.y << bboxYellow.width << bboxYellow.height;
    qDebug() << "bboxorange:" << bboxOrange.x << bboxOrange.y << bboxOrange.width << bboxOrange.height;


    // 노랑/주황 콘 위치 판단
    if (colorName == "yellow" || colorName == "orange") {
        cv::Rect obj = (colorName == "yellow") ? bboxYellow : bboxOrange;

        if (obj.area() == 0) {
            ui->textEdit->append(colorName + ": 없음");
            return;
        }

        int objCenterX = obj.x + obj.width/2;
        int objCenterY = obj.y + obj.height/2;

        int whiteMargin = 100;  // 하얀선 중앙 허용 범위
        int blueMargin  = 100;  // 파란선 중앙 허용 범위

        // x축 기준 (파란선)
        QString verticalPos;
        if (objCenterY < bboxBlue.y - blueMargin) verticalPos = "파란선 위";
        else if (objCenterY > bboxBlue.y + bboxBlue.height + blueMargin) verticalPos = "파란선 아래";
        else verticalPos = "파란선 중앙";

        // y축 기준 (하얀선)
        QString horizontalPos;
        if (objCenterX < bboxWhite.x - whiteMargin) horizontalPos = "하얀선 왼쪽";
        else if (objCenterX > bboxWhite.x + bboxWhite.width + whiteMargin) horizontalPos = "하얀선 오른쪽";
        else horizontalPos = "하얀선 중앙";


        // 최종 위치 판단
        QString finalPos;
        if (horizontalPos == "하얀선 중앙" && verticalPos == "파란선 중앙") finalPos = "중앙";
        else if (horizontalPos == "하얀선 왼쪽" && verticalPos == "파란선 위") finalPos = "2사분면";
        else if (horizontalPos == "하얀선 오른쪽" && verticalPos == "파란선 위") finalPos = "1사분면";
        else if (horizontalPos == "하얀선 왼쪽" && verticalPos == "파란선 아래") finalPos = "3사분면";
        else if (horizontalPos == "하얀선 오른쪽" && verticalPos == "파란선 아래") finalPos = "4사분면";

        else if (horizontalPos == "하얀선 중앙" && verticalPos == "파란선 위") finalPos = "위쪽 하얀선";
        else if (horizontalPos == "하얀선 오른쪽" && verticalPos == "파란선 중앙") finalPos = "오른쪽 파란선";
        else if (horizontalPos == "하얀선 중앙" && verticalPos == "파란선 아래") finalPos = "아래쪽 하얀선";
        else if (horizontalPos == "하얀선 왼쪽" && verticalPos == "파란선 중앙") finalPos = "왼쪽 파란선";
        else finalPos = "없음";

        ui->textEdit->append(colorName + ": " + finalPos);
    }

    int enlargeMargin = 20; // 바운딩 박스 주변 여유 픽셀

    // 노랑
    if (!bboxYellow.empty()) {
        // 여유 적용 후 영역 계산
        int x = std::max(0, bboxYellow.x - enlargeMargin);
        int y = std::max(0, bboxYellow.y - enlargeMargin);
        int w = std::min(frame.cols - x, bboxYellow.width + 2*enlargeMargin);
        int h = std::min(frame.rows - y, bboxYellow.height + 2*enlargeMargin);

        cv::Mat yellowROI = frame(cv::Rect(x, y, w, h)).clone();
        QImage qImgYellow(yellowROI.data, yellowROI.cols, yellowROI.rows, yellowROI.step, QImage::Format_BGR888);
        ui->yellow_con_enlarge->setPixmap(QPixmap::fromImage(qImgYellow).scaled(
            ui->yellow_con_enlarge->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    }

        // 주황
    if (!bboxOrange.empty()) {
        int x = std::max(0, bboxOrange.x - enlargeMargin);
        int y = std::max(0, bboxOrange.y - enlargeMargin);
        int w = std::min(frame.cols - x, bboxOrange.width + 2*enlargeMargin);
        int h = std::min(frame.rows - y, bboxOrange.height + 2*enlargeMargin);

        cv::Mat orangeROI = frame(cv::Rect(x, y, w, h)).clone();
        QImage qImgOrange(orangeROI.data, orangeROI.cols, orangeROI.rows, orangeROI.step, QImage::Format_BGR888);
        ui->orange_con_enlarge->setPixmap(QPixmap::fromImage(qImgOrange).scaled(
            ui->orange_con_enlarge->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    }

}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
    if (event->key() == Qt::Key_Space) {
        onTakePictureButtonClicked();
    }
    QMainWindow::keyPressEvent(event);  // 기본 동작 유지
}

void MainWindow::onTakePictureButtonClicked()
{
    if (!cap.isOpened()) return;

    cap >> frame;
    if (frame.empty()) return;

    clickTakepicture(frame);
}

void MainWindow::clickTakepicture(const Mat &img)
{
    Mat rgb;
    cvtColor(img, rgb, COLOR_BGR2RGB);

    QImage qimg((uchar*)rgb.data, rgb.cols, rgb.rows, rgb.step, QImage::Format_RGB888);
    ui->usb_cam->setPixmap(QPixmap::fromImage(qimg).scaled(ui->usb_cam->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
}

// HSV 슬라이더 변경
void MainWindow::onHLowChanged(int value) {
    if (ui->white_line_2->isChecked()) lower_white_line[0] = value;
    if (ui->blue_line_2->isChecked())  lower_blue_line[0] = value;
    if (ui->yellow_con_2->isChecked()) lower_yellow_con[0] = value;
    if (ui->orange_con_2->isChecked()) lower_orange_con[0] = value;
    ui->H_low_index->setText(QString::number(value));
}

void MainWindow::onHHighChanged(int value) {
    if (ui->white_line_2->isChecked()) upper_white_line[0] = value;
    if (ui->blue_line_2->isChecked())  upper_blue_line[0] = value;
    if (ui->yellow_con_2->isChecked()) upper_yellow_con[0] = value;
    if (ui->orange_con_2->isChecked()) upper_orange_con[0] = value;
    ui->H_high_index->setText(QString::number(value));
}

void MainWindow::onSLowChanged(int value) {
    if (ui->white_line_2->isChecked()) lower_white_line[1] = value;
    if (ui->blue_line_2->isChecked())  lower_blue_line[1] = value;
    if (ui->yellow_con_2->isChecked()) lower_yellow_con[1] = value;
    if (ui->orange_con_2->isChecked()) lower_orange_con[1] = value;
    ui->S_low_index->setText(QString::number(value));
}

void MainWindow::onSHighChanged(int value) {
    if (ui->white_line_2->isChecked()) upper_white_line[1] = value;
    if (ui->blue_line_2->isChecked())  upper_blue_line[1] = value;
    if (ui->yellow_con_2->isChecked()) upper_yellow_con[1] = value;
    if (ui->orange_con_2->isChecked()) upper_orange_con[1] = value;
    ui->S_high_index->setText(QString::number(value));
}

void MainWindow::onKLowChanged(int value) {
    if (ui->white_line_2->isChecked()) lower_white_line[2] = value;
    if (ui->blue_line_2->isChecked())  lower_blue_line[2] = value;
    if (ui->yellow_con_2->isChecked()) lower_yellow_con[2] = value;
    if (ui->orange_con_2->isChecked()) lower_orange_con[2] = value;
    ui->K_low_index->setText(QString::number(value));
}

void MainWindow::onKHighChanged(int value) {
    if (ui->white_line_2->isChecked()) upper_white_line[2] = value;
    if (ui->blue_line_2->isChecked())  upper_blue_line[2] = value;
    if (ui->yellow_con_2->isChecked()) upper_yellow_con[2] = value;
    if (ui->orange_con_2->isChecked()) upper_orange_con[2] = value;
    ui->K_high_index->setText(QString::number(value));
}

// 체크박스 클릭
void MainWindow::onWhiteLineCheckChanged(bool checked) { 
    if (checked) {
        use_white_line = true; use_blue_line = false; use_yellow_con = false; use_orange_con = false;
        ui->blue_line_2->setChecked(false);
        ui->yellow_con_2->setChecked(false);
        ui->orange_con_2->setChecked(false);
        ui->H_low_slider->setValue(lower_white_line[0]);
        ui->H_high_slider->setValue(upper_white_line[0]);
        ui->S_low_slider->setValue(lower_white_line[1]);
        ui->S_high_slider->setValue(upper_white_line[1]);
        ui->K_low_slider->setValue(lower_white_line[2]);
        ui->K_high_slider->setValue(upper_white_line[2]);
    } else use_white_line = false;
}

void MainWindow::onBlueLineCheckChanged(bool checked) { 
    if (checked) {
        use_white_line = false; use_blue_line = true; use_yellow_con = false; use_orange_con = false;
        ui->white_line_2->setChecked(false);
        ui->yellow_con_2->setChecked(false);
        ui->orange_con_2->setChecked(false);
        ui->H_low_slider->setValue(lower_blue_line[0]);
        ui->H_high_slider->setValue(upper_blue_line[0]);
        ui->S_low_slider->setValue(lower_blue_line[1]);
        ui->S_high_slider->setValue(upper_blue_line[1]);
        ui->K_low_slider->setValue(lower_blue_line[2]);
        ui->K_high_slider->setValue(upper_blue_line[2]);
    } else use_blue_line = false;
}

void MainWindow::onYellowConCheckChanged(bool checked) { 
    if (checked) {
        use_white_line = false; use_blue_line = false; use_yellow_con = true; use_orange_con = false;
        ui->white_line_2->setChecked(false);
        ui->blue_line_2->setChecked(false);
        ui->orange_con_2->setChecked(false);
        ui->H_low_slider->setValue(lower_yellow_con[0]);
        ui->H_high_slider->setValue(upper_yellow_con[0]);
        ui->S_low_slider->setValue(lower_yellow_con[1]);
        ui->S_high_slider->setValue(upper_yellow_con[1]);
        ui->K_low_slider->setValue(lower_yellow_con[2]);
        ui->K_high_slider->setValue(upper_yellow_con[2]);
    } else use_yellow_con = false;
}

void MainWindow::onOrangeConCheckChanged(bool checked) { 
    if (checked) {
        use_white_line = false; use_blue_line = false; use_yellow_con = false; use_orange_con = true;
        ui->white_line_2->setChecked(false);
        ui->blue_line_2->setChecked(false);
        ui->yellow_con_2->setChecked(false);
        ui->H_low_slider->setValue(lower_orange_con[0]);
        ui->H_high_slider->setValue(upper_orange_con[0]);
        ui->S_low_slider->setValue(lower_orange_con[1]);
        ui->S_high_slider->setValue(upper_orange_con[1]);
        ui->K_low_slider->setValue(lower_orange_con[2]);
        ui->K_high_slider->setValue(upper_orange_con[2]);
    } else use_orange_con = false;
}

// HSV 리셋
void MainWindow::onHSVResetClicked()
{
    lower_white_line = Scalar(0, 0, 245);
    upper_white_line = Scalar(180, 30, 255);
    lower_blue_line = Scalar(100, 100, 50);
    upper_blue_line = Scalar(130, 255, 255);
    lower_yellow_con = Scalar(26, 160, 100);
    upper_yellow_con = Scalar(30, 255, 255);
    lower_orange_con = Scalar(0, 100, 100);
    upper_orange_con = Scalar(20, 255, 255);

    ui->white_line_2->setChecked(true);
    ui->blue_line_2->setChecked(false);
    ui->yellow_con_2->setChecked(false);
    ui->orange_con_2->setChecked(false);

    use_white_line = true;
    use_blue_line = false;
    use_yellow_con = false;
    use_orange_con = false;

    ui->H_low_slider->setValue(lower_white_line[0]);
    ui->H_high_slider->setValue(upper_white_line[0]);
    ui->S_low_slider->setValue(lower_white_line[1]);
    ui->S_high_slider->setValue(upper_white_line[1]);
    ui->K_low_slider->setValue(lower_white_line[2]);
    ui->K_high_slider->setValue(upper_white_line[2]);
}
