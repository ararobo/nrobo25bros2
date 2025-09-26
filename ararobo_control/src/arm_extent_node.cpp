#include "ararobo_control/arm_extent_node.hpp"
#include <math.h>

ArmExtentNode::ArmExtentNode()
    : Node("arm_extent_node")
{
    // subscribe
    sub_mode_auto_ = this->create_subscription<std_msgs::msg::Bool>(
        "/mode_auto", 10,
        [&](const std_msgs::msg::Bool::SharedPtr msg) -> void
        { mode_auto = msg->data; });
    sub_current_width_ = this->create_subscription<std_msgs::msg::Float32>(
        "/hand/upper/current/width", 10,
        [&](const std_msgs::msg::Float32::SharedPtr msg) -> void
        { current_width = msg->data * diameter; });
    sub_current_depth_ = this->create_subscription<std_msgs::msg::Float32>(
        "/hand/upper/current/depth", 10,
        [&](const std_msgs::msg::Float32::SharedPtr msg) -> void
        { current_depth = msg->data * lead / (2 * M_PI); });
    sub_current_lift_ = this->create_subscription<std_msgs::msg::Float32>(
        "/lift/current/pos", 10,
        [&](const std_msgs::msg::Float32::SharedPtr msg) -> void
        { current_lift = msg->data; });
    sub_box_hold_ = this->create_subscription<std_msgs::msg::Bool>(
        "/box_hold", 10,
        std::bind(&ArmExtentNode::box_hold_callback, this, std::placeholders::_1));
    sub_box_info_ = this->create_subscription<std_msgs::msg::Int8>(
        "/box_info", 10,
        [&](const std_msgs::msg::Int8::SharedPtr msg) -> void
        { box_info = msg->data; });
    // publish
    pub_upper_hand_width_ = this->create_publisher<std_msgs::msg::Float32>(
        "/robot/upper_hand/width", 10);
    pub_upper_hand_depth_ = this->create_publisher<std_msgs::msg::Float32>(
        "/robot/upper_hand/depth", 10);
    pub_centering_vel_ = this->create_publisher<std_msgs::msg::Float32>(
        "/centering_vel", 10);
}

void ArmExtentNode::box_hold_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    box_hold = msg->data; // get info "open or close"

    box_info_converse(box_info, &target_width, &lift_info);

    if (step == 0) // strage
    {
        arm_width = arm_w_max;
        arm_depth = 0.0f;
    }
    else if (step == 1) // open for u_grap
    {
        arm_width = arm_w_max;
        arm_depth = arm_d_max;
    }
    else if (step == 2) // open for l_grap
    {
        arm_width = target_width;
        arm_depth = arm_d_max;
    }
    else if (step == 3)
    {
        flag_ = false;
        if ((box_info == 1 && lift_info) ||
            (box_info == 2 && lift_info) ||
            box_info == 3)
        {
            flag_ = true;
        }
    }
    else if (step == 4) // close for l_grap
    {
        arm_width = target_width;
        arm_depth = 0.1;
    }

    // maximum and minimum limit
    arm_width = clamp(arm_width, arm_w_max, 0.0f);
    arm_depth = clamp(arm_depth, arm_d_max, 0.0f);

    // step complete
    if ((abs(current_width - arm_width) <= error &&
         abs(current_depth - arm_depth) <= error) &&
        flag_)
    {
        ready = true;
    }

    step_update();

    // set width and depth
    upper_hand_width_msg.data = (arm_w_max - arm_width) / diameter / 2.0f * -1;
    upper_hand_depth_msg.data = arm_depth * (2.0f * M_PI) / lead / 2.0f;

    // publish
    pub_upper_hand_width_->publish(upper_hand_width_msg);
    pub_upper_hand_depth_->publish(upper_hand_depth_msg);
    pub_centering_vel_->publish(centering_addend);
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

void ArmExtentNode::box_info_converse(int8_t box_info_, float *box_data, bool *lift_info_)
{
    if (box_info_ == 1) // b_box
    {
        *box_data = box_a_width;
        *lift_info_ = (abs(current_lift - (box_b_width + robot_lift_add)) <= error);
    }
    else if (box_info_ == 2) // c_box upper
    {
        *box_data = box_b_width;
        *lift_info_ = (abs(current_lift - (box_c_width + robot_lift_add)) <= error);
    }
    else if (box_info_ == 3) // c_box lower
    {
        *box_data = box_c_width;
    }
}

void ArmExtentNode::step_update()
{
    if (box_hold) // open
    {
        if (info_save)
        {
            if (step < 4 && ready)
            {
                step++;
            }
        }
        else
        {
            info_save = true;
            step = 0;
        }
    }
    else // close
    {
        if (!info_save)
        {
            if (step > 0 && ready)
            {
                step--;
            }
        }
        else
        {
            info_save = false;
            step = 4;
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
