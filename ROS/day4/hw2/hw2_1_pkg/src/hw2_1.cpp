#include "hw2_1_pkg/hw2_1.hpp"

DS4Teleop::DS4Teleop() : Node("ds4_teleop")
{
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&DS4Teleop::joy_callback, this, std::placeholders::_1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
}

void DS4Teleop::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    geometry_msgs::msg::Twist twist;
    twist.linear.x = msg->axes[1] * 1.0;
    twist.angular.z = msg->axes[3] * 1.0; 
    cmd_pub_->publish(twist);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DS4Teleop>());
    rclcpp::shutdown();
    return 0;
}
