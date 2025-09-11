#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MyCppNode : public rclcpp::Node
{
public:
    MyCppNode();

private:
    void timer_callback();
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mycpp_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int count_ = 0;
};

class MyCppSubscriber : public rclcpp::Node
{
public:
    MyCppSubscriber();

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg);
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mycpp_subscriber_;
};

