#include <QMainWindow>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/srv/set_pen.hpp"
#include <thread>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void onW();
    void onA();
    void onS();
    void onD();
    void onTriangle();
    void onSquare();
    void onCircle();

private:
    void move(double linear, double angular);
    void setPen(int r, int g, int b, int width);
    void drawShape(const std::string &shape);

    Ui::MainWindow *ui;
    rclcpp::Node::SharedPtr node;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;
    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr pen_client;
    std::thread ros_thread;
};
