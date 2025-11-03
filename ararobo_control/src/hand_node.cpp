#include "ararobo_control/hand_node.hpp"

HandNode::HandNode() : Node("hand_node")
{
    pub_upper_depth_ = this->create_publisher<std_msgs::msg::Float32>("/hand/upper/depth", 10);
    pub_upper_width_ = this->create_publisher<std_msgs::msg::Float32>("/hand/upper/width", 10);
    pub_under_slide_ = this->create_publisher<std_msgs::msg::Float32>("/hand/under/slide", 10);
    pub_under_raise_ = this->create_publisher<std_msgs::msg::Float32>("/hand/under/raise", 10);
    pub_hold_cancel_ = this->create_publisher<std_msgs::msg::Bool>("/cancel_hold", 10);
    // pub_box_hold_ = this->create_publisher<std_msgs::msg::Bool>("/hand/box/hold", 10);
    // pub_box_info_ = this->create_publisher<std_msgs::msg::Int8>("/hand/box/info", 10);
    sub_auto_mode_ = this->create_subscription<std_msgs::msg::Bool>(
        "/mode/auto", 10, [&](const std_msgs::msg::Bool::SharedPtr msg)
        { auto_mode = msg->data; });
    sub_upper_position_control_ = this->create_subscription<std_msgs::msg::Bool>(
        "/hand/upper/position_control", 10, [&](const std_msgs::msg::Bool::SharedPtr msg)
        { upper_position_control = msg->data; });
    sub_under_position_control_ = this->create_subscription<std_msgs::msg::Bool>(
        "/hand/under/position_control", 10, [&](const std_msgs::msg::Bool::SharedPtr msg)
        { under_position_control = msg->data; });
    sub_joy_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&HandNode::joy_callback, this, std::placeholders::_1));
    sub_mode_ = this->create_subscription<std_msgs::msg::UInt8>(
        "/phone/mode", 10, [&](const std_msgs::msg::UInt8::SharedPtr msg)
        { mode = msg->data; });
    sub_hold_ = this->create_subscription<std_msgs::msg::Float32>(
        "/phone/upper_hand/depth", 10, [&](const std_msgs::msg::Float32::SharedPtr msg)
        { hold = msg->data; 
          update_hold_ = true; });
    sub_upper_depth_open_limit_ = this->create_subscription<std_msgs::msg::Bool>(
        "/hand/upper/depth/max", 10, [&](const std_msgs::msg::Bool::SharedPtr msg)
        { upper_depth_open_limit = msg->data; });
    sub_upper_depth_close_limit_ = this->create_subscription<std_msgs::msg::Bool>(
        "/hand/upper/depth/min", 10, [&](const std_msgs::msg::Bool::SharedPtr msg)
        { upper_depth_close_limit = msg->data; });
    sub_upper_width_limit_ = this->create_subscription<std_msgs::msg::Bool>(
        "/hand/upper/width/max", 10, [&](const std_msgs::msg::Bool::SharedPtr msg)
        { upper_width_limit = msg->data; });
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&HandNode::timer_callback, this));
}

void HandNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    update_joy_ = true;
    upper_hand_control_velocity_manual(msg);
    under_hand_velocity_control(msg);
}

void HandNode::upper_hand_control_velocity_manual(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    upper_depth = 0.0f;
    upper_width = 0.0f;
    if (upper_hand_in_operation)
    {
        // automatic
        if (operate_mode == 1) // open
        {
            upper_hand_automatic_open();
        }
        else if (operate_mode == 2) // close
        {
            upper_hand_automatic_close();
        }
    }
    else
    {
        if (mode == 1 || mode == 2)
        {
            operate_mode = mode;
            mode = 0;
            upper_hand_in_operation = true;
        }

        // manual
        upper_hand_manual_control(msg);
    }
    if (hold < -0.1f)
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
}

void HandNode::under_hand_velocity_control(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    under_hand_slide = 0.0f;
    under_hand_raise = 0.0f;
    if (under_hand_in_operation)
    {
        if (operate_mode == 3) // open
        {
            under_hand_automatic_open();
        }
        if (operate_mode == 4) // close
        {
            under_hand_automatic_open();
        }
    }
    else
    {
        if (mode == 3 || mode == 4)
        {
            operate_mode = mode;
            mode = 0;
            under_hand_in_operation = true;
        }

        // manual
        under_hand_manual_control(msg);
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
    if (!update_hold_)
    {
        hold = 0.0f;
    }
    update_hold_ = false;
}

void HandNode::upper_hand_automatic_open()
{
    if (!upper_width_limit)
    {
        upper_width += upper_width_speed;
        return;
    }
    if (!upper_depth_open_limit)
    {
        upper_depth += upper_depth_speed;
        return;
    }

    upper_hand_in_operation = false;
}

void HandNode::upper_hand_automatic_close()
{
    if (!upper_width_limit)
    {
        upper_width += upper_width_speed;
        return;
    }
    if (!upper_depth_close_limit)
    {
        upper_depth -= upper_depth_speed;
        return;
    }

    upper_hand_in_operation = false;
}

void HandNode::under_hand_automatic_open()
{
    if (!under_slide_open_limit)
    {
        under_hand_slide += under_hand_slide_speed;
    }
    else
    {
        under_hand_in_operation = false;
    }
}

void HandNode::under_hand_automatic_close()
{
    if (!under_slide_close_limit)
    {
        under_hand_slide -= under_hand_slide_speed;
    }
    else
    {
        under_hand_in_operation = false;
    }
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
