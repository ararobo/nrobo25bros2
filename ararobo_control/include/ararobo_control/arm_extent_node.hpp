#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>
#include "ararobo_msgs/msg/arm_data.hpp"

class ArmExtentNode : public rclcpp::Node
{
private:
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_current_width_;  // current width of arm
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_current_depth_;  // current depth of arm
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_box_info_;       // box_infomation from core
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_width_distance_; // width_distance from box by tof_sensor
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_depth_distance_; // depth_distance from box by tof_sensor
    rclcpp::Publisher<ararobo_msgs::msg::ArmData>::SharedPtr pub_arm_extent_;    // arm_extent to operation
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_centering_vel_;     //

    ararobo_msgs::msg::ArmData arm_extent_msg;          // arm width,depth
    std_msgs::msg::Float32 box_info;                    // ボックスの種類
    std_msgs::msg::Float32 distance_info;               // 距離情報
    std_msgs::msg::Float32 current_width_info;          // 現在のアーム幅
    std_msgs::msg::Float32 current_depth_info;          // 現在のアーム出し入れ
    std_msgs::msg::Float32 current_width_distance_info; // 幅 tof_sensorの距離
    std_msgs::msg::Float32 current_depth_distance_info; // 出し入れ tof_sensorの距離
    std_msgs::msg::Float32 centering_addend;            //

    // constant_data
    float arm_w_max = 650.0;   // 開閉可動域[mm]
    float arm_d_max = 550.0;   // 出し入れ可動域[mm]
    float box_a_width = 300.0; // ボックスAの幅[mm]
    float box_b_width = 400.0; // ボックスBの幅[mm]
    float box_c_width = 500.0; // ボックスCの幅[mm]

    float diameter = 25.46;  // 上アーム 直径[mm](開閉)
    float lead = 30.0;       // 上アーム リード[mm](出し入れ)
    float add_width = 100.0; // 追加のスペース[mm]

    float error = 10.0; // 許容誤差[mm]

    // subscrib_data
    int arm_state = 3;            // 目標動作[収納:0, 開放:1, 把持:2, 現状維持:3]
    int state_s;                  // 目標動作保護
    float target_width;           // ボックス幅[mm]
    float current_width;          // 現在のアーム幅
    float current_depth;          // 現在のアーム出し入れ
    float current_width_distance; // 幅 tof_sensorの距離
    float current_depth_distance; // 出し入れ tof_sensorの距離

    // publish_data
    float arm_width; // 上アーム開閉幅[mm]
    float arm_depth; // 上アーム出し入れ[mm]

    // flag
    int step;                // 手順
    bool ready;              // 手順終了
    bool flag_sensor = true; // tof_sensor距離判定

    void current_width_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void current_depth_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void box_hold_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void width_distance_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void depth_distance_callback(const std_msgs::msg::Float32::SharedPtr msg);

    /**
     * @brief ボックス情報をデータ配列に変換
     * @param box_info ボックス情報
     * @param data     データ配列
     */
    void box_info_converse(float box_info, int *arm_data, float *box_data);

    /**
     * @brief 実行手順の更新
     */
    void step_update();

public:
    ArmExtentNode();
};
