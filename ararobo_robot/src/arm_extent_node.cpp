#include "arm_extent_node.hpp"

ArmExtentNode::ArmExtentNode()
    : Node("arm_extent_node")
{
    // 上アーム開閉幅のサブスクライバー
    sub_arm_width_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/arm_width", 10,
        std::bind(&ArmExtentNode::arm_width_callback, this, std::placeholders::_1));
    // 上アーム出し入れのサブスクライバー
    sub_arm_depth_ = this->create_subscription<std_msgs::msg::Float32>(
        "/arm_depth", 10,
        std::bind(&ArmExtentNode::arm_depth_callback, this, std::placeholders::_1));
}

void ArmExtentNode::arm_width_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
}

void ArmExtentNode::arm_depth_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
}
