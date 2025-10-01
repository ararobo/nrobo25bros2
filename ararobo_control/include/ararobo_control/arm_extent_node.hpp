#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/bool.hpp>

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
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_mode_auto_;        // auto/manual mode
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_current_width_; // current width of arm
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_current_depth_; // current depth of arm
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_current_lift_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_box_hold_; // box_infomation from core
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_box_info_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_upper_hand_width_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_upper_hand_depth_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_centering_vel_;
    rclcpp::TimerBase::SharedPtr timer_;

    // constant_data
    float arm_w_max = 0.010;   // width, range of motion[m]
    float arm_d_max = 0.013;   // depth, range of motion[m]
    float box_a_width = 0.300; // [m]
    float box_b_width = 0.400; // [m]
    float box_c_width = 0.500; // [m]

    float diameter = 0.02546; // width diameter[m]
    float lead = 0.0300;      // depth lesd[m]
    float add_width = 0.100;  // [m]

    float robot_lift_add = 0.010; // feet height

    float error = 0.0100; // Tolerance range[m]

    // subscribe_data
    bool box_hold;       //
    uint8_t box_info;    //
    float target_width;  // [m]
    float current_width; // [m]
    float current_depth; // [m]
    float current_lift;

    // publish_data
    float arm_width; // publish width[mm]
    float arm_depth; // publish depthmm]

    // flag
    int step;
    bool ready;        // step complete
    bool flag_ = true; // lift ready
    bool info_save;
    bool lift_info;
    bool mode_auto = false; // auto/manual mode

    // callback
    void box_hold_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void width_distance_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void timer_callback();

    /**
     * @brief maximum and minimum limit
     *
     * @param[in] variable number of targets
     * @param[in] max      maximum
     * @param[in] min      minimum
     */
    float clamp(float variable, float max, float min);

    /**
     * @brief converse box info
     *
     * @param box_info_  box type
     * @param data       data array
     * @param lift_info_ lift_complete flag
     */
    void box_info_converse(int8_t box_info_, float *box_data, bool *lift_info_);

    /**
     * @brief step update
     */
    void step_update();

public:
    ArmExtentNode();
};
