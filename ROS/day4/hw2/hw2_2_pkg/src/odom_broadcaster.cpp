#include "hw2_2_pkg/odom_broadcaster.hpp"
#include <cmath>

OdomBroadcaster::OdomBroadcaster() : Node("odom_broadcaster"), x_(0), y_(0), theta_(0), linear_vel_(0), angular_vel_(0)
{
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&OdomBroadcaster::cmd_callback, this, std::placeholders::_1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&OdomBroadcaster::update_odom, this));
}

void OdomBroadcaster::cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    linear_vel_ = msg->linear.x;
    angular_vel_ = msg->angular.z;
}

void OdomBroadcaster::update_odom()
{
    double dt = 0.1;
    x_ += linear_vel_ * std::cos(theta_) * dt;
    y_ += linear_vel_ * std::sin(theta_) * dt;
    theta_ += angular_vel_ * dt;

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = this->now();
    odom.header.frame_id = "map";
    odom.child_frame_id = "odom";
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.orientation.z = std::sin(theta_ / 2.0);
    odom.pose.pose.orientation.w = std::cos(theta_ / 2.0);
    odom_pub_->publish(odom);

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->now();
    t.header.frame_id = "map";
    t.child_frame_id = "odom";
    t.transform.translation.x = x_;
    t.transform.translation.y = y_;
    t.transform.translation.z = 0.0;
    t.transform.rotation.z = std::sin(theta_ / 2.0);
    t.transform.rotation.w = std::cos(theta_ / 2.0);
    tf_broadcaster_->sendTransform(t);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomBroadcaster>());
    rclcpp::shutdown();
    return 0;
}
