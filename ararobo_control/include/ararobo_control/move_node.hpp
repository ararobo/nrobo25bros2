#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

class MoveNode : public rclcpp::Node
{
private:
    float linear_x_speed = 1.5f;
    float linear_y_speed = 1.5f;
    float angular_speed = 2.4f;
    uint8_t mode = 0;
    bool auto_mode = false;
    bool acceleration = false;

    bool update_joy_ = false;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

public:
    MoveNode();
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void timer_callback();
};