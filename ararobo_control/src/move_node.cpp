#include "ararobo_control/move_node.hpp"
#include <algorithm>
#include <cmath>

MoveNode::MoveNode() : Node("move_node")
{
    trapezoidal_x = std::make_shared<TrapezoidalController<float>>();
    trapezoidal_y = std::make_shared<TrapezoidalController<float>>();
    trapezoidal_z = std::make_shared<TrapezoidalController<float>>();
    trapezoidal_x->set_control_cycle(10);
    trapezoidal_y->set_control_cycle(10);
    trapezoidal_z->set_control_cycle(10);

    // 時刻の初期化
    last_update_time_ = this->now();

    pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    pub_lift_ = this->create_publisher<std_msgs::msg::Float32>("/lift/target", 10);
    sub_joy_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&MoveNode::joy_callback, this, std::placeholders::_1));
    sub_acceleration_ = this->create_subscription<std_msgs::msg::Bool>(
        "/phone/low_speed", 10, [&](const std_msgs::msg::Bool::SharedPtr msg)
        { low_speed = msg->data; });
    sub_low_speed_ = this->create_subscription<std_msgs::msg::Bool>(
        "/phone/low_accel", 10, [&](const std_msgs::msg::Bool::SharedPtr msg)
        { acceleration = msg->data; });
    sub_phone_lift_ = this->create_subscription<std_msgs::msg::Float32>(
        "/phone/lift", 10, std::bind(&MoveNode::phone_lift_callback, this, std::placeholders::_1));
    timer_communication_check_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&MoveNode::timer_communication_check_callback, this));
}

void MoveNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    update_joy_ = true;
    auto cmd_vel_msg = geometry_msgs::msg::Twist();
    auto lift_msg = std_msgs::msg::Float32();

    // 現在時刻の取得
    rclcpp::Time current_time = this->now();
    double dt = (current_time - last_update_time_).seconds();
    last_update_time_ = current_time;

    // 直線移動
    cmd_vel_msg.linear.x = linear_x_speed * msg->axes[0];
    cmd_vel_msg.linear.y = linear_y_speed * msg->axes[1];

    // 右スティック
    if (std::abs(msg->axes[3]) > angular_stick_threshold) // リフト制御
    {
        // 時間ベースでリフト位置を更新（joyの受信周期に依存しない）
        current_lift_input = msg->axes[3];
        lift_pos += angular_lift_speed * current_lift_input * dt;
        cmd_vel_msg.angular.z = 0.0;
    }
    else // 回転制御
    {
        current_lift_input = 0.0;
        cmd_vel_msg.angular.z = angular_speed * msg->axes[2];
    }
    // 速度制限
    if (low_speed)
    {
        cmd_vel_msg.linear.x = cmd_vel_msg.linear.x * low_speed_rate;
        cmd_vel_msg.linear.y = cmd_vel_msg.linear.y * low_speed_rate;
        cmd_vel_msg.angular.z = cmd_vel_msg.angular.z * low_speed_rate;
    }

    // 台形制御
    if (acceleration)
    {
        cmd_vel_msg.linear.x = cmd_vel_msg.linear.x * low_speed_rate;
        cmd_vel_msg.linear.y = cmd_vel_msg.linear.y * low_speed_rate;
        cmd_vel_msg.angular.z = cmd_vel_msg.angular.z * low_speed_rate;
    }

    pub_cmd_vel_->publish(cmd_vel_msg);

    // リフト位置の制限
    if (lift_pos < -175.0f)
        lift_pos = -175.0f;
    if (lift_pos > 20.0f)
        lift_pos = 20.0f;
    lift_msg.data = lift_pos;
    pub_lift_->publish(lift_msg);
}

void MoveNode::timer_communication_check_callback()
{
    if (!update_joy_)
    {
        RCLCPP_WARN(this->get_logger(), "No joy message received, stopping the robot");
        auto cmd_vel_msg = geometry_msgs::msg::Twist();
        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_msg.linear.y = 0.0;
        cmd_vel_msg.angular.z = 0.0;
        pub_cmd_vel_->publish(cmd_vel_msg);
        return;
    }
    update_joy_ = false;
}

void MoveNode::phone_cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
}

void MoveNode::phone_lift_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MoveNode>());
    rclcpp::shutdown();
    return 0;
}