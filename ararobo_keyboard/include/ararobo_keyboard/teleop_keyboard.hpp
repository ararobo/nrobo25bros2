#pragma once
#include "ararobo_keyboard/linux_keyboard_driver.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

class KeyState
{
private:
    struct keystate
    {
        bool is_pushed;
        bool is_long_pushed;
        uint32_t long_pushed_count;
    };
    KeyState::keystate w, a, s, d;

    void update_state(uint8_t code, uint8_t state);
};

class TeleopKeyboard : public rclcpp::Node
{
private:
    std::shared_ptr<KeyboardDriver> keyboard;
    std::shared_ptr<KeyState> key;
    std::string event_path_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
    double linear_speed_ = 1.0;
    double angular_speed_ = 1.0;
    double shift_rate_ = 0.5;

public:
    TeleopKeyboard(/* args */);
    ~TeleopKeyboard();
    void timer_callback();
};