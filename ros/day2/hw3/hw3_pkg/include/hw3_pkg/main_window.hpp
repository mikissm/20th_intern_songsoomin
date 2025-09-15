#pragma once
#include <QMainWindow>
#include <QLineEdit>
#include <QPushButton>
#include <QLabel>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void onPublishButton();  // 퍼블리시 버튼
private:
    Ui::MainWindow *ui;

    // ROS2
    std::shared_ptr<rclcpp::Node> node;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;
    std::thread ros_thread;

    void rosSpin();
};

