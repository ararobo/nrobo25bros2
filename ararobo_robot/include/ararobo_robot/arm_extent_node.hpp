#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>
#include "ararobo_robot/robot_data_config.hpp"
#include "ararobo_robot/simple_udp.hpp"
#include "ararobo_robot/ethernet_config.hpp"
#include "ararobo_msgs/msg/md_data.hpp"

class ArmExtentNode : public rclcpp::Node
{
private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_arm_width_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_arm_depth_;

    void arm_width_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void arm_depth_callback(const std_msgs::msg::Float32::SharedPtr msg);

public:
    ArmExtentNode();
    ~ArmExtentNode();
};
