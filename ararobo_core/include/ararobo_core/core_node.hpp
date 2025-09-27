#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include "ararobo_core/simple_udp.hpp"
#include "ararobo_core/robot_data_config.hpp"
#include "ararobo_core/trapezoidal_controller.hpp"

class CoreNode : public rclcpp::Node
{
private:
    uint8_t mode = 0;
    bool auto_mode = false;
    bool acceleration = false;
    bool hold = false;
    float lift_pos = 0.0f;
    float upper_depth_speed = 25.0f;
    float upper_width_speed = 1.0f;
    float under_hand_raise_speed = 1.0f;
    float under_hand_slide_speed = 1.0f;
    float under_hand_raise = 0.0f;
    float under_hand_slide = 0.0f;
    float upper_depth_phone = 0.0f;
    // 台形制御
    std::shared_ptr<TrapezoidalController<float>> trapezoidal_controller_x;
    std::shared_ptr<TrapezoidalController<float>> trapezoidal_controller_y;
    std::shared_ptr<TrapezoidalController<float>> trapezoidal_controller_z;
    geometry_msgs::msg::Twist cmd_vel_msg;
    // timer
    rclcpp::TimerBase::SharedPtr timer_;
    // スマホ間通信
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_phone_cmd_vel;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_phone_mode;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_phone_upper_depth;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_phone_acceleration;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_phone_auto_mode;
    // robot
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_robot_cmd_vel;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_robot_upper_depth;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_robot_upper_width;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_robot_under_raise;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_robot_under_slide;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_robot_lift;

    void control_upper_hand_manual();
    void control_under_hand_manual();
    void update_lift_pos(float lift_vel);
    void control_cmd_vel();

public:
    CoreNode();
    void timer_callback();
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void upper_depth_callback(const std_msgs::msg::Float32::SharedPtr msg);
    ~CoreNode();
};