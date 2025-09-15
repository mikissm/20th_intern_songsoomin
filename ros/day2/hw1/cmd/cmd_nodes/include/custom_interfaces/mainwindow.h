#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/add_two_ints.hpp"
#include <vector>
#include <string>

class CmdNode : public rclcpp::Node
{
public:
    CmdNode();
    void run_once(); // CMD 입력 + 퍼블리시

private:
    void topic_callback(const custom_interfaces::msg::AddTwoInts::SharedPtr msg);

    rclcpp::Publisher<custom_interfaces::msg::AddTwoInts>::SharedPtr publisher_;
    rclcpp::Subscription<custom_interfaces::msg::AddTwoInts>::SharedPtr subscription_;
};

#endif // MAINWINDOW_HPP

