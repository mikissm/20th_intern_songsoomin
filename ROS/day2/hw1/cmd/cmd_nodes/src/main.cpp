#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/mainwindow.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CmdNode>();

    while (rclcpp::ok())
    {
        node->run_once();        // CMD 입력 + 퍼블리시
        rclcpp::spin_some(node); // 구독 콜백 처리
    }

    rclcpp::shutdown();
    return 0;
}

