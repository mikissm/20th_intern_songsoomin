#include "hw1_pkg/sub_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/msg/state.hpp"

LifecycleSubscriber::LifecycleSubscriber()
: rclcpp_lifecycle::LifecycleNode("sub_node")
{
}

CallbackReturn LifecycleSubscriber::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Configuring subscriber...");
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    sub_ = this->create_subscription<std_msgs::msg::String>(
        "lifecycle_chatter",
        qos,
        std::bind(&LifecycleSubscriber::callback, this, std::placeholders::_1)
    );
    return CallbackReturn::SUCCESS;
}

CallbackReturn LifecycleSubscriber::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Activating subscriber...");
    return CallbackReturn::SUCCESS;
}

CallbackReturn LifecycleSubscriber::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Deactivating subscriber...");
    return CallbackReturn::SUCCESS;
}

CallbackReturn LifecycleSubscriber::on_cleanup(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Cleaning up subscriber...");
    sub_.reset();
    return CallbackReturn::SUCCESS;
}

void LifecycleSubscriber::callback(std_msgs::msg::String::SharedPtr msg)
{
    if (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        RCLCPP_INFO(get_logger(), "Received: '%s'", msg->data.c_str());
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LifecycleSubscriber>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
