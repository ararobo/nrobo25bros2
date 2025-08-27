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
    sub_box_info_ = this->create_subscription<std_msgs::msg::Float32>(
        "/box_hold", 10,
        std::bind(&ArmExtentNode::box_hold_callback, this, std::placeholders::_1));
    pub_arm_extent_ = this->create_publisher<ararobo_msgs::msg::ArmData>(
        "/arm_target", 10);
    pub_centering_vel_ = this->create_publisher<std_msgs::msg::Float32>(
        "/centering_vel", 10);
}

void ArmExtentNode::box_hold_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    box_info = msg->data; // ボックス情報を取得

    box_info_converse(box_info, &arm_state, &target_width);

    centering_addend.data = 0.0f;

    if (step == 1)
    {
        arm_width = 0.0f;
        arm_depth = 0.0f;
    }
    else if (step == 2)
    {
        arm_width = target_width + add_width;
        arm_depth = 0.0f;
    }
    else if (step == 3)
    {
        arm_width = target_width + add_width;
        arm_depth = target_width + add_width;
    }
    else if (step == 4)
    {
        flag_sensor = abs(current_width_distance - (add_width / 2.0f)) <= error;
        if (!flag_sensor)
        {
            if (current_width_distance - (add_width / 2) > 0.0f)
            {
                centering_addend.data = -(current_width_distance - (add_width / 2.0f));
            }
            else
            {
                centering_addend.data = current_width_distance - (add_width / 2.0f);
            }
        }
    }
    else if (step == 5)
    {
        arm_width = target_width;
        arm_depth = target_width + add_width;
    }
    else if (step == 6)
    {
        flag_sensor = false;
        if (current_depth_distance <= 0.0f)
        {
            flag_sensor = true;
        }
    }
    else if (step == 7)
    {
        // 7
    }

    // 最大,最小値制限 // clamp()
    if (arm_width > arm_w_max)
    {
        arm_width = arm_w_max;
    }
    if (arm_width < 0)
    {
        arm_width = 0;
    }
    if (arm_depth > arm_d_max)
    {
        arm_depth = arm_d_max;
    }
    if (arm_depth < 0)
    {
        arm_depth = 0;
    }

    if ((abs(current_width - arm_width) <= error && abs(current_depth - arm_depth) <= error) && flag_sensor)
    {
        ready = true;
    }

    step_update();

    arm_extent_msg.upper_hand_width = arm_width / diameter / 2.0f;
    arm_extent_msg.upper_hand_depth = arm_depth * (2.0f * M_PI) / lead / 2.0f;

    pub_arm_extent_->publish(arm_extent_msg);
    pub_centering_vel_->publish(centering_addend);
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

void ArmExtentNode::box_info_converse(float box_info, int *arm_data, float *box_data)
{
    // 情報に応じてアーム状態を設定
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

void ArmExtentNode::step_update()
{
    if (arm_state == 0)
    {
        if (state_s == arm_state)
        {
            if (ready)
            {
                ready = false;
                arm_state = 3;
                state_s = 3;
            }
        }
        else
        {
            state_s = arm_state;
            step = 1;
        }
    }
    else if (arm_state == 1)
    {
        if (state_s == arm_state)
        {
            if (ready)
            {
                ready = false;
                if (step < 3)
                {
                    step++;
                }
                else // if (step == 3)
                {
                    arm_state = 3;
                    state_s = 3;
                }
            }
        }
        else
        {
            state_s = arm_state;
            step = 2;
        }
    }
    else if (arm_state == 2)
    {
        if (state_s == arm_state)
        {
            if (ready)
            {
                ready = false;
                if (step < 7)
                {
                    step++;
                }
                else
                {
                    arm_state = 3;
                    state_s = 3;
                }
            }
        }
        else
        {
            state_s = arm_state;
            step = 1;
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
