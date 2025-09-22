#ifndef MAIN_WINDOW_HPP
#define MAIN_WINDOW_HPP

#include <QDebug>
#include <QTimer>
#include <QPushButton>
#include <QLabel>
#include <QImage>
#include <QPixmap>
#include <QCloseEvent>
#include <opencv2/opencv.hpp>
#include "ui_mainwindow.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget* parent = nullptr);
    ~MainWindow();

protected:
    void closeEvent(QCloseEvent* event) override;
    void keyPressEvent(QKeyEvent *event) override;

private slots:
    // 타이머 업데이트
    void upDateImg();

    // 사진 찍기
    void onTakePictureButtonClicked();
    void clickTakepicture(const cv::Mat &img);

    // HSV 슬라이더 변경
    void onHLowChanged(int value);
    void onHHighChanged(int value);
    void onSLowChanged(int value);
    void onSHighChanged(int value);
    void onKLowChanged(int value);
    void onKHighChanged(int value);

    // 체크박스 변경
    void onWhiteLineCheckChanged(bool checked);
    void onBlueLineCheckChanged(bool checked);
    void onYellowConCheckChanged(bool checked);
    void onOrangeConCheckChanged(bool checked);

    // HSV 초기화
    void onHSVResetClicked();

private:
    Ui::MainWindow* ui;

    // 카메라
    cv::VideoCapture cap;
    cv::Mat frame, img_hsv;

    // 마스크
    cv::Mat blue_line_mask, white_line_mask, yellow_con_mask, orange_con_mask;

    // HSV 범위
    cv::Scalar lower_blue_line, upper_blue_line;
    cv::Scalar lower_white_line, upper_white_line;
    cv::Scalar lower_yellow_con, upper_yellow_con;
    cv::Scalar lower_orange_con, upper_orange_con;

    // 사용 여부
    bool use_white_line;
    bool use_blue_line;
    bool use_yellow_con;
    bool use_orange_con;

    // 타이머
    QTimer* updateTimer;

    // 마스크 업데이트
    void updateMask(cv::Mat &mask, const cv::Scalar &lower, const cv::Scalar &upper, QLabel *label);

    // 바운딩 박스
    void drawBoundingBoxes(const cv::Mat &mask, cv::Mat &frame, const QString &colorName);

    cv::Rect bboxYellow;
    cv::Rect bboxOrange;
    cv::Rect bboxBlue;
    cv::Rect bboxWhite;
};

#endif // MAIN_WINDOW_HPP
