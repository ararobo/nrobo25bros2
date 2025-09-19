#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include "ararobo_core/simple_udp.hpp"
#include "ararobo_core/robot_data_config.hpp"

class CoreNode : public rclcpp::Node
{
private:
    std::shared_ptr<SimpleUDP> udp;
    geometry_msgs::msg::Twist cmd_vel_msg;
    uint8_t mode = 0;
    bool acceleration = false;
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
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_acceleration;
    // robot
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_robot_cmd_vel;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_robot_under_right_raise;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_robot_under_right_slide;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_robot_under_left_raise;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_robot_under_left_slide;

public:
    CoreNode();
    void timer_callback();
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void upper_depth_callback(const std_msgs::msg::Float32::SharedPtr msg);
};