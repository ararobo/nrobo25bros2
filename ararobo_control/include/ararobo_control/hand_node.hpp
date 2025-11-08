#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/u_int8.hpp>

class HandNode : public rclcpp::Node
{
private:
    // 速度設定
    float upper_depth_speed = 28.0f;
    float upper_width_speed = 1.0f;
    float under_hand_raise_speed = 1.5f;
    float under_hand_slide_speed = 0.2f;
    float hold_speed = 40.0f;
    // 上ハンド
    float upper_depth = 0.0f;
    float upper_width = 0.0f;
    float under_hand_raise = 0.0f;
    float under_hand_slide = 0.0f;
    uint8_t hand_extent = 0;
    // 動作モード
    bool hold = false;

    // 安全用
    bool update_joy_ = false;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_upper_depth_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_upper_width_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_under_slide_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_under_raise_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_box_hold_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_box_info_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_hold_cancel_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_hand_extent_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_upper_position_control_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_under_position_control_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_hand_extent_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_hold_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_upper_depth_open_limit_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_upper_depth_close_limit_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_upper_width_limit_;
    rclcpp::TimerBase::SharedPtr timer_;

public:
    HandNode();
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void timer_callback();
    void under_hand_velocity_control(const sensor_msgs::msg::Joy::SharedPtr msg);
    void upper_hand_control_velocity_manual(const sensor_msgs::msg::Joy::SharedPtr msg);
    void upper_hand_manual_control(const sensor_msgs::msg::Joy::SharedPtr msg);
    void under_hand_manual_control(const sensor_msgs::msg::Joy::SharedPtr msg);
};