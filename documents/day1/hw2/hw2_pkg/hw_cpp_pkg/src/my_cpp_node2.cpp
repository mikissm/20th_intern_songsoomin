#include "hw2_pkg/my_cpp_node.hpp"
#include <chrono>
using namespace std::chrono_literals;

MyCppNode::MyCppNode() : Node("my_cpp_node")
{
    mycpp_publisher_ = this->create_publisher<std_msgs::msg::String>("helloworld", 10);
    timer_ = this->create_wall_timer(
        1s, std::bind(&MyCppNode::timer_callback, this));
}

void MyCppNode::timer_callback()
{
    auto msg = std_msgs::msg::String();
    msg.data = "Hello World " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Published message: '%s'", msg.data.c_str());
    mycpp_publisher_->publish(msg);
}

MyCppSubscriber::MyCppSubscriber() : Node("my_cpp_subscriber")
{
    mycpp_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "helloworld", 10,
        std::bind(&MyCppSubscriber::topic_callback, this, std::placeholders::_1));
}

void MyCppSubscriber::topic_callback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received message: '%s'", msg->data.c_str());
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto subscriber_node = std::make_shared<MyCppSubscriber>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(subscriber_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}

