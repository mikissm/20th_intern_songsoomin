#include "custom_interfaces/mainwindow.h"
#include <iostream>
#include <sstream>

CmdNode::CmdNode() : Node("cmd_node")
{
    publisher_ = this->create_publisher<custom_interfaces::msg::AddTwoInts>(
        "add_two_ints_topic", 10);

    subscription_ = this->create_subscription<custom_interfaces::msg::AddTwoInts>(
        "add_two_ints_topic", 10,
        std::bind(&CmdNode::topic_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "cmd_node started!");
}

void CmdNode::run_once()
{
    int64_t a_input;
    std::string line;
    std::vector<int32_t> b_input;

    // a 입력
    while (true)
    {
        std::cout << "Enter a (int64): ";
        std::getline(std::cin, line);
        std::istringstream iss(line);
        if (iss >> a_input)
        {
            break; 
        }
        else
        {
            std::cout << "Invalid input. Please enter an integer." << std::endl;
        }
    }

    // b 벡터 입력
    while (true)
    {
        std::cout << "Enter b vector (space-separated ints, end with ENTER): ";
        std::getline(std::cin, line);
        std::istringstream iss(line);
        b_input.clear();
        int num;
        bool valid = true;

        while (iss >> num)
        {
            b_input.push_back(num);
        }

        if (b_input.empty())
        {
            std::cout << "Vector cannot be empty. Try again." << std::endl;
            continue;
        }

        if (iss.fail() && !iss.eof())
        {
            std::cout << "Invalid input detected. Only integers are allowed." << std::endl;
            continue;
        }

        break; 
    }

    custom_interfaces::msg::AddTwoInts msg;
    msg.a = a_input;
    msg.b = b_input;

    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published a=%ld, b_size=%zu", msg.a, msg.b.size());
}


void CmdNode::topic_callback(const custom_interfaces::msg::AddTwoInts::SharedPtr msg)
{
    std::cout << "Received: a=" << msg->a << ", b=[";
    for (size_t i = 0; i < msg->b.size(); ++i)
    {
        std::cout << msg->b[i];
        if (i < msg->b.size() - 1)
            std::cout << ", ";
    }
    std::cout << "]" << std::endl;
}

