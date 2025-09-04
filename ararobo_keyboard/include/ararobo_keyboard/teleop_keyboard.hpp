#pragma once
#include "ararobo_keyboard/linux_keyboard_driver.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>
#include <rclcpp/rclcpp.hpp>

class KeyState
{
public:
    struct keystate
    {
        bool is_pushed;
        bool is_long_pushed;
        uint32_t long_pushed_count;
    };
    KeyState::keystate w, a, s, d, t, f, g, h, up, down, left, right, y, u, j, i, k, o, l, p, n, m, shift, x;

    void update_state(uint8_t code, uint8_t state);
};

class TeleopKeyboard : public rclcpp::Node
{
private:
    std::shared_ptr<KeyboardDriver> keyboard;
    std::shared_ptr<KeyState> key;
    std::string event_path_ = "/dev/input/event3";
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_upper_hand_width_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_upper_hand_depth_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_lift_;
    double linear_speed_ = 1.0;
    double angular_speed_ = 2.0;
    double shift_rate_ = 0.4;
    double hand_depth_speed_ = 15.0;
    double hand_width_speed_ = 1.0;
    double lift_speed_ = 20.0;
    double raise_speed_ = 0.5;
    double slide_speed_ = 0.5;

    bool hold_flag_ = false;

public:
    TeleopKeyboard(/* args */);
    ~TeleopKeyboard();
    void timer_callback();
};