#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>

/*
float32 upper_hand_width
float32 upper_hand_depth
float32 right_slide
float32 right_raise
float32 left_slide
float32 left_raise
*/
class ArmExtentNode : public rclcpp::Node
{
private:
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_current_width_;  // current width of arm
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_current_depth_;  // current depth of arm
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_width_distance_; // width_distance from box by tof_sensor
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_depth_distance_; // depth_distance from box by tof_sensor
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_box_info_;       // box_infomation from core
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_upper_hand_width_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_upper_hand_depth_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_centering_vel_;
    // ararobo_msgs::msg::ArmData

    std_msgs::msg::Float32 upper_hand_width_msg;
    std_msgs::msg::Float32 upper_hand_depth_msg;
    std_msgs::msg::Float32 centering_addend;

    // constant_data
    float arm_w_max = 0.650;   // 開閉可動域[m]
    float arm_d_max = 0.550;   // 出し入れ可動域[m]
    float box_a_width = 0.300; // ボックスAの幅[m]
    float box_b_width = 0.400; // ボックスBの幅[m]
    float box_c_width = 0.500; // ボックスCの幅[m]

    float diameter = 0.02546; // 上アーム 直径[m](開閉)
    float lead = 0.0300;      // 上アーム リード[m](出し入れ)
    float add_width = 0.100;  // 追加のスペース[m]

    float error = 0.0100; // 許容誤差[m]

    // subscribe_data
    float box_info;                      // ボックスに関する情報
    int arm_state = 3;                   // 目標動作[収納:0, 開放:1, 把持:2, 現状維持:3]
    int state_s;                         // 目標動作保護
    float target_width;                  // ボックス幅[m]
    float current_width;                 // 現在のアーム幅
    float current_depth;                 // 現在のアーム出し入れ
    float current_width_distance = 0.0f; // 幅 tof_sensorの距離
    float current_depth_distance = 0.0f; // 出し入れ tof_sensorの距離

    // publish_data
    float arm_width; // 上アーム開閉幅[mm]
    float arm_depth; // 上アーム出し入れ[mm]

    // flag
    int step;                // 手順
    bool ready;              // 手順終了
    bool flag_sensor = true; // tof_sensor距離判定

    // callback
    void current_width_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void current_depth_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void box_hold_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void width_distance_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void depth_distance_callback(const std_msgs::msg::Float32::SharedPtr msg);

    float clamp(float variable, float max, float min);

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
