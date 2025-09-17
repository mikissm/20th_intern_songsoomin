#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/srv/set_pen.hpp"
#include <thread>
#include <termios.h>
#include <unistd.h>

class TurtleDrawer : public rclcpp::Node
{
public:
    TurtleDrawer();

private:
    void keyboard_loop();
    void draw_shape(const std::string &shape);
    void set_pen(int r, int g, int b, int width);

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr pen_client_;
    std::thread keyboard_thread_;
};
