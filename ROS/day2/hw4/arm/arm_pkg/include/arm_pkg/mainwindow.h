#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QGraphicsRectItem>
#include <QGraphicsScene>
#include <QTimer>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

public slots:
    void updateRotation();
    void setArmAngles(double base, double shoulder, double elbow);

    void rotateBaseClockwise();
    void rotateBaseCounterClockwise();
    void stopBase();

    void rotateShoulderClockwise();
    void rotateShoulderCounterClockwise();
    void stopShoulder();

    void rotateElbowClockwise();
    void rotateElbowCounterClockwise();
    void stopElbow();

    void saveState();
    void loadState();

private:
    Ui::MainWindow *ui;

    QGraphicsScene *scene;
    QGraphicsRectItem *baseArm;
    QGraphicsRectItem *shoulderArm;
    QGraphicsRectItem *elbowArm;

    int baseDir;
    int shoulderDir;
    int elbowDir;

    QTimer *timer;

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
    std::thread rosThread_;
    bool rosRunning_ = true;
};

#endif // MAINWINDOW_H

