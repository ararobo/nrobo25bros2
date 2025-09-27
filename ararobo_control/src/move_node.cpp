#include "ararobo_control/move_node.hpp"

MoveNode::MoveNode() : Node("move_node")
{
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&MoveNode::joy_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&MoveNode::timer_callback, this));
}

void MoveNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    update_joy_ = true;
    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x = linear_x_speed * msg->axes[1];
    twist.linear.y = linear_y_speed * msg->axes[0];
    twist.angular.z = angular_speed * msg->axes[2];
    cmd_vel_pub_->publish(twist);
}

void MoveNode::timer_callback()
{
    if (!update_joy_)
    {
        auto twist = geometry_msgs::msg::Twist();
        twist.linear.x = 0.0;
        twist.linear.y = 0.0;
        twist.angular.z = 0.0;
        cmd_vel_pub_->publish(twist);
        return;
    }
    update_joy_ = false;
}