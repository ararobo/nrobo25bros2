#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>
#include "ararobo_robot/robot_data_config.hpp"
#include "ararobo_robot/ethernet_config.hpp"
#include "ararobo_msgs/msg/arm_data.hpp"

class ArmExtentNode : public rclcpp::Node
{
private:
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_current_width_; //
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_current_depth_; //
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_box_info_;      // box_infomation from core
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_distance_;      // distance from box by tof_sensor
    rclcpp::Publisher<ararobo_msgs::msg::ArmData>::SharedPtr pub_arm_extent_;   // arm_extent to operation

    ararobo_msgs::msg::ArmData arm_extent_msg;
    std_msgs::msg::Float32 box_info;           // ボックスの種類
    std_msgs::msg::Float32 distance_info;      // 距離情報
    std_msgs::msg::Float32 current_width_info; //
    std_msgs::msg::Float32 current_depth_info; //

    // constant_data
    float u_arm_w_max = 650;   // 開閉可動域[mm]
    float u_arm_d_max = 550;   // 出し入れ可動域[mm]
    float box_a_width = 300.0; // ボックスAの幅[mm]
    float box_b_width = 400.0; // ボックスBの幅[mm]
    float box_c_width = 500.0; // ボックスCの幅[mm]

    float diameter = 25.46; // 上アーム 直径[mm](開閉)
    float lead = 30.0;      // 上アーム リード[mm](出し入れ)

    float error = 10.0;      // 許容誤差[mm]
    float add_width = 100.0; // 追加のスペース

    // subscribed_data
    float arm_state;     // 目標動作[収納:0, 開放:1, 把持:2]
    float target_width;  // ボックス幅[mm]
    float current_width; // 現在のアーム幅
    float current_depth; // 現在のアーム出し入れ

    // published_data
    float arm_width; // 上アーム開閉幅[mm]
    float arm_depth; // 上アーム出し入れ[mm]

    // flag
    int step_s1 = 0; // 開放動作のステップ[初期:0, 開閉:1, 出し入れ:2]
    int step_s2 = 0; // 把持動作のステップ

    void current_width_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void current_depth_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void box_hold_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void distance_callback(const std_msgs::msg::Float32::SharedPtr msg);

    /**
     * @brief ボックス情報をデータ配列に変換
     * @param box_info ボックス情報
     * @param data     データ配列
     */
    void box_info_converse(float box_info, float *arm_data, float *box_data);

public:
    ArmExtentNode();
    ~ArmExtentNode();
};
