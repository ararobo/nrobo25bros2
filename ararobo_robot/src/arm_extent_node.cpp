#include "arm_extent_node.hpp"
#include <math.hpp>

ArmExtentNode::ArmExtentNode()
    : Node("arm_extent_node")
{
    sub_current_width_ = this->create_subscription<std_msgs::msg::Float32>(
        "/current_width", 10,
        std::bind(&ArmExtentNode::current_width_callback, this, std::placeholders::_1));
    sub_current_depth_ = this->create_subscription<std_msgs::msg::Float32>(
        "/current_depth", 10,
        std::bind(&ArmExtentNode::current_depth_callback, this, std::placeholders::_1));
    sub_box_info_ = this->create_subscription<std_msgs::msg::Float32>(
        "/box_hold", 10,
        std::bind(&ArmExtentNode::box_hold_callback, this, std::placeholders::_1));
    sub_distance_ = this->create_subscription<std_msgs::msg::Float32>(
        "/distance", 10,
        std::bind(&ArmExtentNode::distance_callback, this, std::placeholders::_1));
    pub_arm_extent_ = this->create_publisher<ararobo_msgs::msg::ArmData>(
        "/arm_target", 10);
}

void ArmExtentNode::box_hold_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    box_info = *msg; // ボックス情報を取得

    box_info_converse(box_info.data, &arm_state, &target_width);

    if (arm_state == 0)
    {
        if (step_s0 == 0)
        {
            step_s0 = 1;
            arm_width = 0.0;
            arm_depth = 0.0;
        }
    }
    else if (arm_state == 1)
    {
        if (step_s1 == 0)
        {
            step_s1 = 1;
            arm_width = target_width + 100;
        }
        if (step_s1 == 2)
        {
            step_s1 = 3;
            arm_depth = target_width + 100;
        }
    }
    else if (arm_state == 2)
    {
        if (step_s2 == 0)
        {
            step_s2 = 1;
            arm_width = target_width;
        }
        if (step_s2 == 2)
        {
            step_s2 = 3;
            arm_depth = target_width;
        }
        if (step_s2 == 4)
        {
            step_s2 = 5;
        }
    }

    arm_extent_msg.width = arm_width / diameter / 2;
    arm_extent_msg.depth = arm_depth * (2 * M_PI) / lead / 2;

    pub_arm_extent_->publish(arm_extent_msg);
}

void ArmExtentNode::distance_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    distance_info = *msg; // 距離情報を取得
    // 位置微調整の処理
}

void ArmExtentNode::current_width_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    current_width_info = *msg;
    current_width = current_width_info.data * diameter;
    if (arm_state == 0)
    {
        if (step_s1 == 1 && abs(current_width) >= error)
        {
            step_s1 = 0;
        }
    }
    else if (arm_state == 1)
    {
        if (step_s1 == 1 && abs(current_width - target_width) >= error)
        {
            step_s1 = 2; // 2 -> 3
        }
        else if (step_s1 == 3 && abs(current_depth - target_width) >= error)
        {
            step_s1 = 0; // 3/
        }
    }
    else if (arm_state == 2)
    {
        if (step_s2 == 1 && abs(current_width - target_width) >= error)
        {
            step_s1 = 2; // 5 -> 6
        }
        else if (step_s2 == 3 && abs(current_depth - target_width) >= error)
        {
            step_s1 = 4; // 6 -> 7
        }
        else if (step_s2 == 5 && abs(current_depth - target_width) >= error)
        {
            step_s1 = 0; // 7/
        }
    }
}

void ArmExtentNode::current_depth_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    current_depth_info = *msg;
    current_depth = current_depth_info.data * lead / (2 * M_PI);
}

void ArmExtentNode::box_info_converse(float box_info, float *arm_data, float *box_data)
{
    // ボックス情報に応じてアーム状態を設定
    if (box_info == 0)
    {
        *arm_data = 0; // strage
    }
    else if (box_info == 1 || box_info == 2 || box_info == 3)
    {
        *arm_data = 1; // open
    }
    else if (box_info == 6 || box_info == 7 || box_info == 8)
    {
        *arm_data = 2; // grab
    }
    // ボックスの種類に応じて幅を設定
    if (box_info == 1 || box_info == 6)
    {
        *box_data = box_a_width; // box A
    }
    else if (box_info == 2 || box_info == 7)
    {
        *box_data = box_b_width; // box B
    }
    else if (box_info == 3 || box_info == 8)
    {
        *box_data = box_c_width; // box C
    }
}

void ArmExtentNode::state_0()
{
}

void ArmExtentNode::state_1()
{
}
