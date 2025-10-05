#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include "ararobo_control/trapezoidal_controller.hpp"

class MoveNode : public rclcpp::Node
{
private:
    // 速度設定
    float linear_x_speed = 1.5f;
    float linear_y_speed = 1.5f;
    float angular_speed = -2.4f;
    float angular_lift_speed = 0.5f;
    float angular_stick_threshold = 0.5f;
    float max_acceleration = 1.0f;
    // リフト位置制御
    bool lift_position_control = false;
    float lift_pos = 0.0f;
    // 動作モード
    uint8_t mode = 0;
    bool auto_mode = false;
    bool acceleration = false;
    // 安全用
    bool update_joy_ = false;

    // 台形制御
    std::shared_ptr<TrapezoidalController<float>> trapezoidal_x;
    std::shared_ptr<TrapezoidalController<float>> trapezoidal_y;
    std::shared_ptr<TrapezoidalController<float>> trapezoidal_z;
    // Publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_lift_;
    // Subscription
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_auto_mode_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_acceleration_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_lift_position_control_;
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

public:
    MoveNode();
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void timer_callback();
};