#include "ararobo_control/arm_extent_node.hpp"
#include <math.h>

ArmExtentNode::ArmExtentNode()
    : Node("arm_extent_node")
{
    sub_current_width_ = this->create_subscription<std_msgs::msg::Float32>(
        "/current_width", 10,
        std::bind(&ArmExtentNode::current_width_callback, this, std::placeholders::_1));
    sub_current_depth_ = this->create_subscription<std_msgs::msg::Float32>(
        "/current_depth", 10,
        std::bind(&ArmExtentNode::current_depth_callback, this, std::placeholders::_1));
    sub_width_distance_ = this->create_subscription<std_msgs::msg::Float32>(
        "/width_distance", 10,
        std::bind(&ArmExtentNode::width_distance_callback, this, std::placeholders::_1));
    sub_depth_distance_ = this->create_subscription<std_msgs::msg::Float32>(
        "/depth_distance", 10,
        std::bind(&ArmExtentNode::depth_distance_callback, this, std::placeholders::_1));
    sub_box_hold_ = this->create_subscription<std_msgs::msg::Bool>(
        "/box_hold", 10,
        std::bind(&ArmExtentNode::box_hold_callback, this, std::placeholders::_1));
    sub_box_info_ = this->create_subscription<std_msgs::msg::Int8>(
        "/box_info", 10,
        std::bind(&ArmExtentNode::box_info_callback, this, std::placeholders::_1));
    pub_upper_hand_width_ = this->create_publisher<std_msgs::msg::Float32>(
        "/upper_hand/width", 10);
    pub_upper_hand_depth_ = this->create_publisher<std_msgs::msg::Float32>(
        "/upper_hand/depth", 10);
    pub_centering_vel_ = this->create_publisher<std_msgs::msg::Float32>(
        "/centering_vel", 10);
}

void ArmExtentNode::box_hold_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    box_hold = msg->data; // ボックス情報を取得

    box_info_converse(box_info, &target_width);

    if (step == 1) // strage
    {
        arm_width = arm_w_max;
        arm_depth = 0.0f;
    }
    else if (step == 2) // open for u_grap
    {
        arm_width = arm_w_max;
        arm_depth = arm_d_max;
    }
    else if (step == 3) // open for l_grap
    {
        arm_width = target_width;
        arm_depth = arm_d_max;
    }
    else if (step == 4) // close for l_grap
    {
        arm_width = target_width;
        arm_depth = 0.1;
    }

    // 最大,最小値制限
    arm_width = clamp(arm_width, arm_w_max, 0.0f);
    arm_depth = clamp(arm_depth, arm_d_max, 0.0f);

    if ((abs(current_width - arm_width) <= error && abs(current_depth - arm_depth) <= error) && flag_)
    {
        ready = true;
    }

    step_update();

    upper_hand_width_msg.data = (arm_w_max - arm_width) / diameter / 2.0f * -1;
    upper_hand_depth_msg.data = arm_depth * (2.0f * M_PI) / lead / 2.0f;

    pub_upper_hand_width_->publish(upper_hand_width_msg);
    pub_upper_hand_depth_->publish(upper_hand_depth_msg);
    pub_centering_vel_->publish(centering_addend);
}

void ArmExtentNode::box_info_callback(const std_msgs::msg::Int8::SharedPtr msg)
{
    box_info = msg->data;
}

void ArmExtentNode::current_width_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    current_width = msg->data * diameter;
}

void ArmExtentNode::current_depth_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    current_depth = msg->data * lead / (2 * M_PI);
}

void ArmExtentNode::width_distance_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    current_width_distance = msg->data;
}

void ArmExtentNode::depth_distance_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    current_depth_distance = msg->data;
}

float ArmExtentNode::clamp(float variable, float max, float min)
{
    if (variable > max)
    {
        return max;
    }
    else if (variable < min)
    {
        return min;
    }
    else
    {
        return variable;
    }
}

void ArmExtentNode::box_info_converse(int8_t box_info_, float *box_data)
{
    // old_model
    if (box_info_ == 1)
    {
        *box_data = box_a_width;
    }
    else if (box_info_ == 2)
    {
        *box_data = box_b_width;
    }
    else if (box_info_)
    {
        *box_data = box_c_width;
    }
}

void ArmExtentNode::step_update()
{ // old model
    if (box_info)
    {
        if (box_info && info_save)
        {
            if (step < 4 /*&& (step == 1 && lift~~)*/)
            {
                step++;
            }
        }
        else
        {
            info_save == box_info;
            step == 1;
        }
    }
    else
    {
        if (box_info && info_save)
        {
            if (step > 1 /*&& (step == 2 && lift~~)*/)
            {
                step--;
            }
        }
        else
        {
            info_save == box_info;
            step == 4;
        }
    }
}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmExtentNode>());
    rclcpp::shutdown();
    return 0;
}
