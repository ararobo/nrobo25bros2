#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "ararobo_robot/simple_udp.hpp"
#include "ararobo_robot/robot_data_config.hpp"
#include "ararobo_robot/ethernet_config.hpp"
#include "ararobo_robot/odom_calculator.hpp"

class FeedbackNode : public rclcpp::Node
{
private:
    double encoder_resolution; // エンコーダの分解能
    double wheel_radius;       // 車輪の半径
    uint8_t period_odom;       // オドメトリ計算周期[ms]

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_current_hand_width_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_current_hand_depth_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_current_lift_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_distance_right_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_distance_left_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_position_control_lift_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_position_control_upper_hand_depth_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_position_control_upper_hand_width_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_position_control_under_hand_right_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_position_control_under_hand_left_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_wheel_encoder_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<SimpleUDP> udp;
    std::shared_ptr<OdomCalculator> odom_calculator;

    union feedback_data_union_t
    {
        feedback_data_t data;                  // フィードバックデータ
        uint8_t code[sizeof(feedback_data_t)]; // 送信バイト配列
    } __attribute__((__packed__)) feedback_union;

    double x = 0.0;          // x座標
    double y = 0.0;          // y座標
    double theta = 0.0;      // ヨー角
    double prev_theta = 0.0; // 前回のヨー角

    rclcpp::Time prev_time; // 最後のオドメトリ計算時間

    void timer_callback();

public:
    FeedbackNode();
    ~FeedbackNode();
};