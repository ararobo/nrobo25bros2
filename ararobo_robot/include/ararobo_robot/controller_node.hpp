#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include "ararobo_robot/simple_udp.hpp"
#include "ararobo_robot/robot_data_config.hpp"
#include "ararobo_robot/ethernet_config.hpp"

class ControllerNode : public rclcpp::Node
{
private:
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr pub_joy_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_connection_status_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<SimpleUDP> controller_udp;
    std::shared_ptr<SimpleUDP> mainboard_udp;

    union controller_data_union_t
    {
        uint8_t code[sizeof(controller_data_t)];
        controller_data_t data;
    } controller_union;

    bool is_controller_connected = false;
    bool is_mainboard_connected = false;
    uint32_t controller_disconnect_count = 0;
    uint32_t controller_disconnect_threshold = 50; // 50 * 10ms = 500ms
    uint32_t mainboard_disconnect_count = 0;
    uint32_t mainboard_disconnect_threshold = 50; // 50 * 10ms = 500ms

    uint32_t mainboard_tick = 0;
    uint32_t controller_tick = 0;
    uint32_t mainboard_tick_prev = 0;
    uint32_t controller_tick_prev = 0;
    uint32_t mainboard_tick_threshold = 15;
    uint32_t controller_tick_threshold = 15;

public:
    ControllerNode();
    ~ControllerNode();
    void timer_callback();
};