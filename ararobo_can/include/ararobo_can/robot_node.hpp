#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "ararobo_can/pc_data_slave.hpp"

class RobotNode : public rclcpp::Node
{
private:
    std::shared_ptr<PCDataSlave> pc_data_slave_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

public:
    RobotNode();
    ~RobotNode();
};
