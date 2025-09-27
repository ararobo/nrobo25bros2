#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "ararobo_robot/simple_udp.hpp"
#include "ararobo_robot/robot_data_config.hpp"
#include "ararobo_robot/ethernet_config.hpp"

class ControllerNode : public rclcpp::Node
{
private:
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr pub_joy_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<SimpleUDP> controller_udp;
    std::shared_ptr<SimpleUDP> mainboard_udp;

    union controller_data_union_t
    {
        uint8_t code[sizeof(controller_data_t)];
        controller_data_t data;
    } controller_union;

    bool is_controller_connected = false;
    uint32_t disconnect_count = 0;
    uint32_t disconnect_threshold = 100; // 100 * 10ms = 1s

public:
    ControllerNode();
    ~ControllerNode();
    void timer_callback();
};