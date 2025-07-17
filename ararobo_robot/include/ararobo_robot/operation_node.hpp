#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include "ararobo_robot/robot_data_config.hpp"
#include "ararobo_robot/simple_udp.hpp"
#include "ararobo_robot/ethernet_config.hpp"
#include "ararobo_msgs/msg/md_data.hpp"

class OperationNode : public rclcpp::Node
{
private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
    rclcpp::Subscription<ararobo_msgs::msg::MdData>::SharedPtr sub_md_data_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<SimpleUDP> udp;

    operation_data_t operation_data;

    union operation_data_union_t
    {
        operation_data_t data;                  // 操作データ
        uint8_t code[sizeof(operation_data_t)]; // 送信バイト配列
    } __attribute__((__packed__)) operation_union;

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

    void md_data_callback(const ararobo_msgs::msg::MdData::SharedPtr msg);

    void timer_callback();

public:
    OperationNode();
    ~OperationNode();
};