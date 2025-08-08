#include "arm_extent_node.hpp"
#include <math.h>

ArmExtentNode::ArmExtentNode()
    : Node("arm_extent_node")
{
    sub_box_info_ = this->create_subscription<std_msgs::msg::Float32>(
        "/box_hold", 10,
        std::bind(&ArmExtentNode::box_hold_callback, this, std::placeholders::_1));
    sub_distance_ = this->create_subscription<std_msgs::msg::Float32>(
        "/distance", 10,
        std::bind(&ArmExtentNode::distance_callback, this, std::placeholders::_1));
    pub_arm_extent_ = this->create_publisher<ararobo_msgs::msg::ArmData>(
        "/arm_width", 10);
}

void ArmExtentNode::box_hold_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    box_info = *msg; // ボックス情報を取得

    box_info_converse(box_info.data, extent_data);

    arm_extent_msg.width = extent_data[0] / diameter / 2;
    arm_extent_msg.depth = extent_data[1] * 2 * M_PI / lead / 2;

    pub_arm_extent_->publish(arm_extent_msg);
}

void ArmExtentNode::distance_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    distance_info = *msg; // 距離情報を取得
    // 位置微調整の処理
}

void ArmExtentNode::box_info_converse(float box_info, float data[2])

{
    switch (int(box_info))
    {
    case 0: // ハンド収納
        data[0] = 0.0f;
        data[1] = 0.0f;
        break;
    case 1: // 開放(A)
        data[0] = box_a_width + 100.0f;
        data[1] = 0.0f;
        break;
    case 2: // 開放(B)
        data[0] = box_b_width + 100.0f;
        data[1] = 0.0f;
        break;
    case 3: // 開放(C)
        data[0] = box_c_width + 100.0f;
        data[1] = 0.0f;
        break;
    case 6: // 把持(A)
        data[0] = box_a_width;
        data[1] = 0.0f;
        break;
    case 7: // 把持(B)
        data[0] = box_b_width;
        data[1] = 0.0f;
        break;
    case 8: // 把持(C)
        data[0] = box_c_width;
        data[1] = 0.0f;
        break;
    default:
        break;
    }
    if (data[0] > u_arm_w_max)
    {
        data[0] = u_arm_w_max; // 上アーム最大開閉幅制限
    }
    if (data[1] > u_arm_d_max)
    {
        data[1] = u_arm_d_max; // 上アーム最大出し入れ制限
    }
}