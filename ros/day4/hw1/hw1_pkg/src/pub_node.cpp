#include "hw1_pkg/pub_node.hpp"
#include "rclcpp/rclcpp.hpp"

LifecyclePublisher::LifecyclePublisher()
: rclcpp_lifecycle::LifecycleNode("pub_node")
{
}

CallbackReturn LifecyclePublisher::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Configuring publisher...");
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    pub_ = this->create_publisher<std_msgs::msg::String>("lifecycle_chatter", qos);

    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&LifecyclePublisher::timer_callback, this)
    );
    timer_->cancel(); // 아직 active가 아니므로 타이머는 중지
    return CallbackReturn::SUCCESS;
}

CallbackReturn LifecyclePublisher::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Activating publisher...");
    pub_->on_activate();
    timer_->reset(); // 타이머 재시작
    return CallbackReturn::SUCCESS;
}

CallbackReturn LifecyclePublisher::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Deactivating publisher...");
    pub_->on_deactivate();
    timer_->cancel();
    return CallbackReturn::SUCCESS;
}

CallbackReturn LifecyclePublisher::on_cleanup(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Cleaning up publisher...");
    pub_.reset();
    timer_.reset();
    return CallbackReturn::SUCCESS;
}

void LifecyclePublisher::timer_callback()
{
    if (pub_->is_activated()) {
        auto msg = std_msgs::msg::String();
        msg.data = "Hello from lifecycle publisher";
        RCLCPP_INFO(get_logger(), "Publishing: '%s'", msg.data.c_str());
        pub_->publish(msg);
    }
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LifecyclePublisher>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
