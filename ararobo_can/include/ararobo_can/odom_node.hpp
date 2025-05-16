#pragma once
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "ararobo_can/pc_data_slave.hpp"

class OdomNode : public rclcpp::Node
{
private:
    std::shared_ptr<PCDataSlave> pc_data_slave_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry_;
    rclcpp::TimerBase::SharedPtr timer_;
    float x, y, theta; // 位置と角度

    void timer_callback();

public:
    OdomNode();
    ~OdomNode();
};
