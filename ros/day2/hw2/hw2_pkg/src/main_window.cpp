#include "hw2_pkg/main_window.hpp"
#include "ui_MainWindow.h"
#include <QPushButton>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    rclcpp::init(0, nullptr);
    node = std::make_shared<rclcpp::Node>("turtle_qt_controller");
    cmd_pub = node->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
    pen_client = node->create_client<turtlesim::srv::SetPen>("turtle1/set_pen");

    connect(ui->ww, &QPushButton::clicked, this, &MainWindow::onW);
    connect(ui->aa, &QPushButton::clicked, this, &MainWindow::onA);
    connect(ui->ss, &QPushButton::clicked, this, &MainWindow::onS);
    connect(ui->dd, &QPushButton::clicked, this, &MainWindow::onD);
    connect(ui->btn1, &QPushButton::clicked, this, &MainWindow::onTriangle);
    connect(ui->btn2, &QPushButton::clicked, this, &MainWindow::onSquare);
    connect(ui->btn3, &QPushButton::clicked, this, &MainWindow::onCircle);

    ros_thread = std::thread([this]() {
        rclcpp::Rate rate(10);
        while (rclcpp::ok()) {
            rclcpp::spin_some(node);
            rate.sleep();
        }
    });
}

MainWindow::~MainWindow()
{
    rclcpp::shutdown();
    if (ros_thread.joinable()) ros_thread.join();
    delete ui;
}

void MainWindow::move(double linear, double angular)
{
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = linear;
    msg.angular.z = angular;
    cmd_pub->publish(msg);
}

void MainWindow::setPen(int r, int g, int b, int width)
{
    if (!pen_client->wait_for_service(1s)) return;
    auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
    request->r = r; request->g = g; request->b = b;
    request->width = width; request->off = 0;
    pen_client->async_send_request(request);
}

void MainWindow::drawShape(const std::string &shape)
{
    if (shape == "triangle") {
        setPen(255,0,0,3);
        for(int i=0;i<3;i++){
            move(2.0,0.0); std::this_thread::sleep_for(1s);
            move(0.0,2.094); std::this_thread::sleep_for(1s);
        }
    } else if (shape == "square") {
        setPen(0,255,0,5);
        for(int i=0;i<4;i++){
            move(2.0,0.0); std::this_thread::sleep_for(1s);
            move(0.0,1.57); std::this_thread::sleep_for(1s);
        }
    } else if (shape == "circle") {
        setPen(0,0,255,2);
        for(int i=0;i<7;i++){
            move(3.0,5); std::this_thread::sleep_for(0.2s);
        }
    }
    move(0.0,0.0);
}

void MainWindow::onW() { move(2.0,0.0); }
void MainWindow::onS() { move(-2.0,0.0); }
void MainWindow::onA() { move(0.0,1.5); }
void MainWindow::onD() { move(0.0,-1.5); }
void MainWindow::onTriangle() { drawShape("triangle"); }
void MainWindow::onSquare() { drawShape("square"); }
void MainWindow::onCircle() { drawShape("circle"); }

