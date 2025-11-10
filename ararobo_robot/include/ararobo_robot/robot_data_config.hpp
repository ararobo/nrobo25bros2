/**
 * @file robot_data_config.hpp
 * @author aiba-gento
 * @brief ロボットの通信データ構造体定義
 * @version 2.1
 * @date 2025-10-03
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once
#include <stdint.h>
#include "led_config.hpp"

constexpr uint16_t operation_data_header = 0xAB36;
constexpr uint16_t feedback_data_header = 0x554A;
constexpr uint16_t controller_data_header = 0x15A5;
constexpr uint16_t pid_gain_data_header = 0x5A5C;

struct feedback_data_t
{
    uint16_t header;                        // ヘッダー
    uint32_t tick;                          // 送信時のタイムスタンプ
    int16_t encoder_x;                      // x軸方向のエンコーダ値[CRP]
    int16_t encoder_y;                      // y軸方向のエンコーダ値[CRP]
    uint16_t distance_base_rf;              // 右前の距離センサ[mm]
    uint16_t distance_base_rm;              // 右中央の距離センサ[mm]
    uint16_t distance_base_rb;              // 右後の距離センサ[mm]
    uint16_t distance_base_lf;              // 左前の距離センサ[mm]
    uint16_t distance_base_lm;              // 左中央の距離センサ[mm]
    uint16_t distance_base_lb;              // 左後の距離センサ[mm]
    bool position_control_lift;             // 昇降機構位置制御モードか否か
    bool position_control_upper_hand_depth; // 上ハンド機構奥行き位置制御モードか否か
    bool position_control_upper_hand_width; // 上ハンド機構幅位置制御モードか否か
    float current_lift;                     // 昇降機構位置制御現在値
    float upper_hand_current_depth;         // 上ハンド機構位置制御現在値
    float upper_hand_current_width;         // 上ハンド機構位置制御現在値
    bool limit_depth_max;
    bool limit_depth_min;
    bool limit_width_max;
    bool limit_width_min;
} __attribute__((__packed__));

struct operation_data_t
{
    uint16_t header;         // ヘッダー
    led_config_t led_config; // LED設定データ
    float vx;                // x軸方向の速度[m/s]
    float vy;                // y軸方向の速度[m/s]
    float omega;             // 回転速度[rad/s]
    float lift;              // 昇降
    float upper_hand_width;  // 上ハンド開閉幅
    float upper_hand_depth;  // 上ハンド出し入れ
    float under_arm_slide;   // 下アーム前後スライド
    float under_arm_raise;   // 下アーム上下回転
    uint8_t hand_extent;     // 0:未操作 1:展開 2:収納
} __attribute__((__packed__));

struct controller_data_t
{
    uint16_t header;      // ヘッダー
    uint32_t tick;        // 送信時のタイムスタンプ
    int8_t left_stick_x;  // 左スティック左右
    int8_t left_stick_y;  // 左スティック上下
    int8_t right_stick_x; // 右スティック左右
    int8_t right_stick_y; // 右スティック上下
    struct buttons
    {
        uint8_t l_up : 1;
        uint8_t l_down : 1;
        uint8_t l_left : 1;
        uint8_t l_right : 1;
        uint8_t r_up : 1;
        uint8_t r_down : 1;
        uint8_t r_left : 1;
        uint8_t r_right : 1;
    } __attribute__((__packed__)) buttons;
} __attribute__((__packed__));

struct pid_gain_t
{
    uint16_t header;   // ヘッダー
    uint8_t id;        // MDのID
    bool reinitialize; // 再初期化するか否か
    float p;           // 比例ゲイン
    float i;           // 積分ゲイン
    float d;           // 微分ゲイン
} __attribute__((__packed__));
