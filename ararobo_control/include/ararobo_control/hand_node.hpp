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
    float upper_depth_speed = 25.0f;
    float upper_width_speed = 1.0f;
    float under_hand_raise_speed = 1.0f;
    float under_hand_slide_speed = 1.0f;
    float hold_speed = 25.0f;
    // 位置制御時速度
    float upper_depth_pos_speed = 20.0f;
    float upper_width_pos_speed = 1.0f;
    // 上ハンド
    float upper_depth = 0.0f;
    float upper_width = 0.0f;
    float under_hand_raise = 0.0f;
    float under_hand_slide = 0.0f;
    // 動作モード
    uint8_t mode = 0;
    bool auto_mode = false;
    bool upper_position_control = false;
    bool under_position_control = false;
    float hold = 0.0f;
    bool upper_hand_in_operation;
    bool under_hand_in_operation;
    int operate_mode; // 1:open , 2:close
    // リミットスイッチ
    bool upper_width_limit = false;
    bool upper_depth_open_limit = false;
    bool upper_depth_close_limit = false;
    bool under_raise_open_limit = false;
    bool under_raise_close_limit = false;
    bool under_slide_open_limit = false;
    bool under_slide_close_limit = false;

    // 安全用
    bool update_joy_ = false;
    bool update_hold_ = false;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_upper_depth_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_upper_width_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_under_slide_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_under_raise_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_box_hold_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_box_info_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_hold_cancel_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_auto_mode_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_upper_position_control_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_under_position_control_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_mode_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_hold_;
    rclcpp::TimerBase::SharedPtr timer_;

public:
    HandNode();
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void timer_callback();
    void under_hand_velocity_control(const sensor_msgs::msg::Joy::SharedPtr msg);
    void upper_hand_control_velocity_manual(const sensor_msgs::msg::Joy::SharedPtr msg);
    void upper_hand_automatic_open();
    void upper_hand_automatic_close();
    void upper_hand_automatic_release();
    void under_hand_automatic_open();
    void under_hand_automatic_close();
    void upper_hand_manual_control(const sensor_msgs::msg::Joy::SharedPtr msg);
    void under_hand_manual_control(const sensor_msgs::msg::Joy::SharedPtr msg);
};