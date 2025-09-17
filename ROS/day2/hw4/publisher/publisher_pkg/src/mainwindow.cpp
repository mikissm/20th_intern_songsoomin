#include "publisher_pkg/mainwindow.h"
#include "ui_mainwindow.h"
#include <QSlider>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // ROS2 초기화
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("arm_publisher_gui_node");
    pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("arm_angles", 10);

    rosThread_ = std::thread([this]() {
        rclcpp::Rate rate(30);
        while (rclcpp::ok() && rosRunning_) {
            rclcpp::spin_some(node_);
            rate.sleep();
        }
    });

    // 슬라이더가 바뀔 때 라벨 업데이트
    connect(ui->baseSlider, &QSlider::valueChanged, this, &MainWindow::updateLabels);
    connect(ui->shoulderSlider, &QSlider::valueChanged, this, &MainWindow::updateLabels);
    connect(ui->elbowSlider, &QSlider::valueChanged, this, &MainWindow::updateLabels);

    // 퍼블리시 버튼 클릭 시 메시지 전송
    connect(ui->publishButton, &QPushButton::clicked, this, &MainWindow::publishAngles);

    // 초기 라벨 세팅
    updateLabels();
}

MainWindow::~MainWindow()
{
    rosRunning_ = false;
    if (rosThread_.joinable()) rosThread_.join();
    rclcpp::shutdown();
    delete ui;
}

// 퍼블리시 버튼 클릭 시 호출
void MainWindow::publishAngles()
{
    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data = {
        static_cast<double>(ui->baseSlider->value()),
        static_cast<double>(ui->shoulderSlider->value()),
        static_cast<double>(ui->elbowSlider->value())
    };
    pub_->publish(msg);
}

// 슬라이더 값 기반 라벨 갱신
void MainWindow::updateLabels()
{
    ui->baseLabel->setText(QString("Base: %1°").arg(ui->baseSlider->value()));
    ui->shoulderLabel->setText(QString("Shoulder: %1°").arg(ui->shoulderSlider->value()));
    ui->elbowLabel->setText(QString("Elbow: %1°").arg(ui->elbowSlider->value()));
}

