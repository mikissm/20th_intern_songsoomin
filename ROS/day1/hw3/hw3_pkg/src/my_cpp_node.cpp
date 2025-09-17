#include "hw3_pkg/my_cpp_node.hpp"
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

TurtleDrawer::TurtleDrawer() : Node("turtle_drawer")
{
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
    pen_client_ = this->create_client<turtlesim::srv::SetPen>("turtle1/set_pen");

    keyboard_thread_ = std::thread(&TurtleDrawer::keyboard_loop, this);
}

void TurtleDrawer::set_pen(int r, int g, int b, int width)
{
    if (!pen_client_->wait_for_service(1s)) {
        RCLCPP_WARN(this->get_logger(), "SetPen service not available");
        return;
    }

    auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
    request->r = r;
    request->g = g;
    request->b = b;
    request->width = width;
    request->off = 0;

    pen_client_->async_send_request(request);
}

void TurtleDrawer::keyboard_loop()
{
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    geometry_msgs::msg::Twist msg;

    while (rclcpp::ok()) {
        char c = getchar();
        msg.linear.x = 0.0;
        msg.angular.z = 0.0;

        if (c == 'w') msg.linear.x = 2.0;
        else if (c == 's') msg.linear.x = -2.0;
        else if (c == 'a') msg.angular.z = 1.5;
        else if (c == 'd') msg.angular.z = -1.5;
        else if (c == '1') draw_shape("triangle");
        else if (c == '2') draw_shape("square");
        else if (c == '3') draw_shape("circle");
        else if (c == 'q') break;

        cmd_pub_->publish(msg);
        rclcpp::sleep_for(100ms);
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
}

void TurtleDrawer::draw_shape(const std::string &shape)
{
    geometry_msgs::msg::Twist msg;
    if (shape == "triangle") {
        set_pen(255, 0, 0, 3);
        for (int i = 0; i < 3; i++) {
            msg.linear.x = 2.0;
            msg.angular.z = 0.0;
            cmd_pub_->publish(msg);
            rclcpp::sleep_for(1s);

            msg.linear.x = 0.0;
            msg.angular.z = 2.094;
            cmd_pub_->publish(msg);
            rclcpp::sleep_for(1s);
        }
    }
    else if (shape == "square") {
        set_pen(0, 255, 0, 5);
        for (int i = 0; i < 4; i++) {
            msg.linear.x = 2.0;
            msg.angular.z = 0.0;
            cmd_pub_->publish(msg);
            rclcpp::sleep_for(1s);

            msg.linear.x = 0.0;
            msg.angular.z = 1.57;
            cmd_pub_->publish(msg);
            rclcpp::sleep_for(1s);
        }
    }
    else if (shape == "circle") {
        set_pen(0, 0, 255, 2);
        for (int i = 0; i < 3; i++) {
            msg.linear.x = 2.0;
            msg.angular.z = 3;
            cmd_pub_->publish(msg);
            rclcpp::sleep_for(1s);
        }
    }

    msg.linear.x = 0.0;
    cmd_pub_->publish(msg);
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleDrawer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
