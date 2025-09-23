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
    std::shared_ptr<SimpleUDP> udp;
    std::shared_ptr<TrapezoidalController<float>> trapezoidal_controller_x;
    std::shared_ptr<TrapezoidalController<float>> trapezoidal_controller_y;
    std::shared_ptr<TrapezoidalController<float>> trapezoidal_controller_z;
    geometry_msgs::msg::Twist cmd_vel_msg;
    uint8_t mode = 0;
    bool auto_mode = false;
    bool acceleration = false;
    bool hold = false;
    float lift_vel = 0.0f;
    float lift_pos = 0.0f;
    float upper_depth_speed = 20.0f;
    float upper_width_speed = 1.0f;
    float under_hand_raise_speed = 1.0f;
    float under_hand_slide_speed = 1.0f;
    union controller_data_union_t
    {
        uint8_t code[sizeof(controller_data_t)];
        controller_data_t data;
    } controller_union;

    // timer
    rclcpp::TimerBase::SharedPtr timer_;
    // phone auto
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_phone_cmd_vel;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_phone_mode;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_phone_upper_depth;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_phone_acceleration;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_phone_auto_mode;
    // robot
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_robot_cmd_vel;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_robot_under_raise;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_robot_under_slide;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_robot_lift;

public:
    CoreNode();
    void timer_callback();
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void upper_depth_callback(const std_msgs::msg::Float32::SharedPtr msg);
    ~CoreNode();
};