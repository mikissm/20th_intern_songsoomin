#include "hw3_pkg/main_window.hpp"
#include "ui_MainWindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // ROS2 초기화
    rclcpp::init(0, nullptr);
    node = std::make_shared<rclcpp::Node>("qt_talker_listener");

    // 퍼블리셔
    pub = node->create_publisher<std_msgs::msg::String>("chatter", 10);

    // 서브스크라이버
    sub = node->create_subscription<std_msgs::msg::String>(
        "chatter", 10,
        [this](std_msgs::msg::String::SharedPtr msg) {
            // GUI 스레드 안전하게 라벨 업데이트
            QString text = QString::fromStdString(msg->data);
            QMetaObject::invokeMethod(ui->label, "setText",
                                      Qt::QueuedConnection,
                                      Q_ARG(QString, text));
        });

    // 버튼 클릭 연결
    connect(ui->pushButton, &QPushButton::clicked, this, &MainWindow::onPublishButton);

    // ROS2 spin 별도 스레드
    ros_thread = std::thread([this]() { rosSpin(); });
}

MainWindow::~MainWindow()
{
    rclcpp::shutdown();
    if (ros_thread.joinable()) ros_thread.join();
    delete ui;
}

void MainWindow::onPublishButton()
{
    auto msg = std_msgs::msg::String();
    msg.data = ui->lineEdit->text().toStdString();
    pub->publish(msg);
}

void MainWindow::rosSpin()
{
    rclcpp::Rate rate(10);
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        rate.sleep();
    }
}

