#include "ararobo_control/hand_node.hpp"

HandNode::HandNode() : Node("hand_node")
{
    pub_upper_depth_ = this->create_publisher<std_msgs::msg::Float32>("/hand/upper/depth", 10);
    pub_upper_width_ = this->create_publisher<std_msgs::msg::Float32>("/hand/upper/width", 10);
    pub_under_slide_ = this->create_publisher<std_msgs::msg::Float32>("/hand/under/slide", 10);
    pub_under_raise_ = this->create_publisher<std_msgs::msg::Float32>("/hand/under/raise", 10);
    pub_hold_cancel_ = this->create_publisher<std_msgs::msg::Bool>("/cancel_hold", 10);
    pub_hand_extent_ = this->create_publisher<std_msgs::msg::UInt8>("/hand/extent", 10);
    sub_hand_extent_ = this->create_subscription<std_msgs::msg::UInt8>(
        "/phone/mode", 10, [&](const std_msgs::msg::UInt8::SharedPtr msg)
        { hand_extent = msg->data; });
    sub_joy_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&HandNode::joy_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&HandNode::timer_callback, this));
}

void HandNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    update_joy_ = true;
    if (msg->buttons[0] || msg->buttons[1] || msg->buttons[2] || msg->buttons[3] ||
        msg->buttons[4] || msg->buttons[5] || msg->buttons[6])
    {
        hand_extent = 0;
    }
    if (msg->buttons[7])
    {
        hand_extent = 1;
    }
    if (msg->buttons[5])
    {
        hold = true;
    }
    if (msg->buttons[0])
    {
        hold = false;
    }

    upper_hand_control_velocity_manual(msg);
    under_hand_velocity_control(msg);
}

void HandNode::upper_hand_control_velocity_manual(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    upper_depth = 0.0f;
    upper_width = 0.0f;
    // manual
    upper_hand_manual_control(msg);
    // hold
    if (hold)
    {
        upper_depth = -hold_speed;
        if (msg->buttons[0])
        {
            auto hold_cancel_msg = std_msgs::msg::Bool();
            hold_cancel_msg.data = true;
            pub_hold_cancel_->publish(hold_cancel_msg);
        }
    }
    std_msgs::msg::Float32 upper_depth_msg;
    upper_depth_msg.data = upper_depth;
    pub_upper_depth_->publish(upper_depth_msg);
    std_msgs::msg::Float32 upper_width_msg;
    upper_width_msg.data = upper_width;
    pub_upper_width_->publish(upper_width_msg);
    std_msgs::msg::UInt8 hand_extent_msg;
    hand_extent_msg.data = hand_extent;
    pub_hand_extent_->publish(hand_extent_msg);
}

void HandNode::under_hand_velocity_control(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    under_hand_slide = 0.0f;
    under_hand_raise = 0.0f;

    under_hand_manual_control(msg);
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
        std_msgs::msg::Float32 upper_depth_msg;
        upper_depth_msg.data = 0.0;
        pub_upper_depth_->publish(upper_depth_msg);
        std_msgs::msg::Float32 upper_width_msg;
        upper_width_msg.data = 0.0;
        pub_upper_width_->publish(upper_width_msg);
        std_msgs::msg::Float32 under_slide_msg;
        under_slide_msg.data = 0.0;
        pub_under_slide_->publish(under_slide_msg);
        std_msgs::msg::Float32 under_raise_msg;
        under_raise_msg.data = 0.0;
        pub_under_raise_->publish(under_raise_msg);
    }
    update_joy_ = false;
}

void HandNode::upper_hand_manual_control(const sensor_msgs::msg::Joy::SharedPtr msg)
{
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

void HandNode::under_hand_manual_control(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    if (msg->buttons[4])
    {
        under_hand_raise += under_hand_raise_speed;
    }
    if (msg->buttons[5])
    {
        under_hand_raise -= under_hand_raise_speed;
    }
    if (msg->buttons[6])
    {
        under_hand_slide -= under_hand_slide_speed;
    }
    if (msg->buttons[7])
    {
        under_hand_slide += under_hand_slide_speed;
    }
}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HandNode>());
    rclcpp::shutdown();
    return 0;
}
