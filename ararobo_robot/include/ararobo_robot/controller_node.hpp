#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include "ararobo_robot/simple_udp.hpp"
#include "ararobo_robot/robot_data_config.hpp"
#include "ararobo_robot/ethernet_config.hpp"
#include <regex>

class ControllerNode : public rclcpp::Node
{
private:
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr pub_joy_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_connection_status_;
    rclcpp::TimerBase::SharedPtr recv_timer_;
    rclcpp::TimerBase::SharedPtr ping_timer_;
    std::shared_ptr<SimpleUDP> controller_udp[2];
    std::shared_ptr<SimpleUDP> mainboard_udp;
    std::string target_ip_address = "192.168.2.111";

    union controller_data_union_t
    {
        uint8_t code[sizeof(controller_data_t)];
        controller_data_t data;
    } controller_union[3];

    bool is_controller_connected = false;
    bool is_mainboard_connected = false;
    uint32_t controller_disconnect_count = 0;
    uint32_t controller_disconnect_threshold = 50; // 50 * 10ms = 500ms
    uint32_t mainboard_disconnect_count = 0;
    uint32_t mainboard_disconnect_threshold = 50; // 50 * 10ms = 500ms
    double avg_ping_time = -1.0;                  // 平均応答時間（ms）、-1は未測定
    double ping_time_threshold = 150.0;           // 応答時間の閾値（ms）

public:
    ControllerNode();
    ~ControllerNode();
    void recv_timer_callback();
    void ping_timer_callback();
    void publish_joy(controller_data_t &controller_data);
    double get_ping_time();
};