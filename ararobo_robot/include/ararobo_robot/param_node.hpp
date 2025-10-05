#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include "ararobo_robot/simple_udp.hpp"
#include "ararobo_robot/robot_data_config.hpp"
#include "ararobo_robot/ethernet_config.hpp"

class ParamNode : public rclcpp::Node
{
private:
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_pid_gain_;
    std::shared_ptr<SimpleUDP> udp;
    int motor_id = 1; // モーターID（0~11）
    union pid_gain_union_t
    {
        pid_gain_t data;                  // PIDゲインデータ
        uint8_t code[sizeof(pid_gain_t)]; // 送信バイト配列
    } __attribute__((__packed__)) pid_gain_union;

public:
    ParamNode();
    ~ParamNode();
    void pid_gain_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
};