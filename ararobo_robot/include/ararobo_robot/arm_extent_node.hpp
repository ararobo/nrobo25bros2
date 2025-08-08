#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>
#include "ararobo_robot/robot_data_config.hpp"
#include "ararobo_robot/ethernet_config.hpp"
#include "ararobo_msgs/msg/md_data.hpp"

class ArmExtentNode : public rclcpp::Node
{
private:
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_box_info_;   // box_infomation from core
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_distance_;   // distance from box by tof_sensor
    rclcpp::Publisher<ararobo_msgs::msg::MdData>::SharedPtr pub_arm_extent_; // arm_extent to operation

    void box_hold_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void distance_callback(const std_msgs::msg::Float32::SharedPtr msg);

    ararobo_msgs::msg::MdData arm_extent_msg;
    std_msgs::msg::Float32 box_info;      // ボックス情報
    std_msgs::msg::Float32 distance_info; // 距離情報

    // data
    float u_arm_w_max; // 上アーム最大開閉幅[mm]
    float u_arm_d_max; // 上アーム最大出し入れ[mm]
    float box_a_width; // ボックスAの幅[mm]
    float box_b_width; // ボックスBの幅[mm]
    float box_c_width; // ボックスCの幅[mm]
    float box_d_width; // ボックスDの幅[mm]
    float box_e_width; // ボックスEの幅[mm]

    float diameter = 25.46; // 上アームの直径[mm](幅)
    float lead;             // 上アームのリード[mm](出し入れ)

    // substitute
    float arm_width;      // 上アーム開閉幅[rad]
    float arm_depth;      // 上アーム出し入れ[rad]
    float extent_data[2]; // 上アーム開閉幅と出し入れのデータ配列(width, depth)[mm]

    /**
     * @brief ボックス情報をデータ配列に変換
     * @param box_info ボックス情報
     * @param data データ配列
     */
    void box_info_converse(float box_info, float data[2]);

public:
    ArmExtentNode();
    ~ArmExtentNode();
};
