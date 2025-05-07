#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "ararobo_can/can_driver.hpp"

class RobotNode : public rclcpp::Node
{
private:
    std::shared_ptr<CANDriver> can_driver_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;

    uint16_t can_id_cmd_vel_;
    int cmd_vel_scale_;

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

public:
    RobotNode();
    ~RobotNode();
};
