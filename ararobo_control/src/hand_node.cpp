#include "ararobo_control/hand_node.hpp"

HandNode::HandNode() : Node("hand_node")
{
    pub_upper_depth_ = this->create_publisher<std_msgs::msg::Float32>("/hand/upper/depth", 10);
    pub_upper_width_ = this->create_publisher<std_msgs::msg::Float32>("/hand/upper/width", 10);
    pub_under_slide_ = this->create_publisher<std_msgs::msg::Float32>("/hand/under/slide", 10);
    pub_under_raise_ = this->create_publisher<std_msgs::msg::Float32>("/hand/under/raise", 10);
    sub_auto_mode_ = this->create_subscription<std_msgs::msg::Bool>(
        "/mode/auto", 10, [&](const std_msgs::msg::Bool::SharedPtr msg)
        { auto_mode = msg->data; });
    sub_upper_position_control_ = this->create_subscription<std_msgs::msg::Bool>(
        "/hand/upper/position_control", 10, [&](const std_msgs::msg::Bool::SharedPtr msg)
        { upper_position_control = msg->data; });
    sub_under_position_control_ = this->create_subscription<std_msgs::msg::Bool>(
        "/hand/under/position_control", 10, [&](const std_msgs::msg::Bool::SharedPtr msg)
        { under_position_control = msg->data; });
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&HandNode::timer_callback, this));
    sub_joy_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&HandNode::joy_callback, this, std::placeholders::_1));
}

void HandNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    update_joy_ = true;
    if (auto_mode)
    {
    }
    else
    {
        upper_hand_control_manual(msg);
    }
    under_hand_control(msg);
}

void HandNode::upper_hand_control_manual(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    if (upper_position_control)
    {
    }
    else
    {
        upper_depth = 0.0f;
        upper_width = 0.0f;
        if (msg->buttons[0])
        {
            upper_depth += upper_depth_speed;
        }
        if (msg->buttons[1])
        {
            upper_depth -= upper_depth_speed;
        }
        if (msg->buttons[2])
        {
            upper_width -= upper_width_speed;
        }
        if (msg->buttons[3])
        {
            upper_width += upper_width_speed;
        }
    }

    std_msgs::msg::Float32 upper_depth_msg;
    upper_depth_msg.data = upper_depth;
    pub_upper_depth_->publish(upper_depth_msg);
    std_msgs::msg::Float32 upper_width_msg;
    upper_width_msg.data = upper_width;
    pub_upper_width_->publish(upper_width_msg);
}

void HandNode::under_hand_control(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    if (under_position_control)
    {
    }
    else
    {
        under_hand_slide = 0.0f;
        under_hand_raise = 0.0f;
        if (msg->buttons[4])
        {
            under_hand_slide += under_hand_slide_speed;
        }
        if (msg->buttons[5])
        {
            under_hand_slide -= under_hand_slide_speed;
        }
        if (msg->buttons[6])
        {
            under_hand_raise += under_hand_raise_speed;
        }
        if (msg->buttons[7])
        {
            under_hand_raise -= under_hand_raise_speed;
        }
    }

    std_msgs::msg::Float32 under_slide_msg;
    under_slide_msg.data = under_hand_slide;
    pub_under_slide_->publish(under_slide_msg);
    std_msgs::msg::Float32 under_raise_msg;
    under_raise_msg.data = under_hand_raise;
    pub_under_raise_->publish(under_raise_msg);
}

void HandNode::timer_callback()
{
    if (!update_joy_)
    {
        if (!upper_position_control)
        {
            std_msgs::msg::Float32 upper_depth_msg;
            upper_depth_msg.data = 0.0;
            pub_upper_depth_->publish(upper_depth_msg);
            std_msgs::msg::Float32 upper_width_msg;
            upper_width_msg.data = 0.0;
            pub_upper_width_->publish(upper_width_msg);
        }
        if (!under_position_control)
        {
            std_msgs::msg::Float32 under_slide_msg;
            under_slide_msg.data = 0.0;
            pub_under_slide_->publish(under_slide_msg);
            std_msgs::msg::Float32 under_raise_msg;
            under_raise_msg.data = 0.0;
            pub_under_raise_->publish(under_raise_msg);
        }
    }
    update_joy_ = false;
}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HandNode>());
    rclcpp::shutdown();
    return 0;
}
