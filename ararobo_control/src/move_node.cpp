#include "ararobo_control/move_node.hpp"

MoveNode::MoveNode() : Node("move_node")
{
    trapezoidal_x = std::make_shared<TrapezoidalController<float>>();
    trapezoidal_y = std::make_shared<TrapezoidalController<float>>();
    trapezoidal_z = std::make_shared<TrapezoidalController<float>>();
    trapezoidal_x->set_control_cycle(10);
    trapezoidal_y->set_control_cycle(10);
    trapezoidal_z->set_control_cycle(10);
    pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    pub_lift_ = this->create_publisher<std_msgs::msg::Float32>("/lift/target", 10);
    sub_joy_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&MoveNode::joy_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&MoveNode::timer_callback, this));
}

void MoveNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    update_joy_ = true;
    auto cmd_vel_msg = geometry_msgs::msg::Twist();
    auto lift_msg = std_msgs::msg::Float32();
    // 直線移動
    cmd_vel_msg.linear.x = linear_x_speed * msg->axes[0];
    cmd_vel_msg.linear.y = linear_y_speed * msg->axes[1];
    // 右スティック
    if (std::abs(msg->axes[3]) > angular_stick_threshold) // リフト制御
    {
        lift_pos += angular_lift_speed * msg->axes[3];
        cmd_vel_msg.angular.z = 0.0;
    }
    else // 回転制御
    {
        cmd_vel_msg.angular.z = angular_speed * msg->axes[2];
    }
    // 台形制御
    if (acceleration)
    {
        cmd_vel_msg.linear.x = trapezoidal_x->trapezoidal_control(cmd_vel_msg.linear.x, max_acceleration);
        cmd_vel_msg.linear.y = trapezoidal_y->trapezoidal_control(cmd_vel_msg.linear.y, max_acceleration);
        cmd_vel_msg.angular.z = trapezoidal_z->trapezoidal_control(cmd_vel_msg.angular.z, max_acceleration);
    }

    pub_cmd_vel_->publish(cmd_vel_msg);

    lift_pos = std::clamp(lift_pos, -160.0f, 0.0f);
    lift_msg.data = lift_pos;
    pub_lift_->publish(lift_msg);
}

void MoveNode::timer_callback()
{
    if (!update_joy_)
    {
        auto cmd_vel_msg = geometry_msgs::msg::Twist();
        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_msg.linear.y = 0.0;
        cmd_vel_msg.angular.z = 0.0;
        pub_cmd_vel_->publish(cmd_vel_msg);
        return;
    }
    update_joy_ = false;
}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MoveNode>());
    rclcpp::shutdown();
    return 0;
}