/**
 * @file robot_data_config.hpp
 * @author aiba-gento
 * @brief ロボットの通信データ構造体定義
 * @version 1.1
 * @date 2025-09-26
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once
#include <stdint.h>

struct feedback_data_t
{
    uint32_t tick;                    // 送信時のタイムスタンプ
    float q_x;                        // ロボットの回転クォータニオン
    float q_y;                        // ロボットの回転クォータニオン
    float q_z;                        // ロボットの回転クォータニオン
    float q_w;                        // ロボットの回転クォータニオン
    int16_t encoder_x;                // x軸方向のエンコーダ値[CRP]
    int16_t encoder_y;                // y軸方向のエンコーダ値[CRP]
    uint16_t distance_base_rf;        // 右前の距離センサ[mm]
    uint16_t distance_base_rm;        // 右中央の距離センサ[mm]
    uint16_t distance_base_rb;        // 右後の距離センサ[mm]
    uint16_t distance_base_lf;        // 左前の距離センサ[mm]
    uint16_t distance_base_lm;        // 左中央の距離センサ[mm]
    uint16_t distance_base_lb;        // 左後の距離センサ[mm]
    float upper_hand_current_depth;   // 上ハンド機構位置制御現在値
    float upper_hand_current_width;   // 上ハンド機構位置制御現在値
    float current_lift;               // 昇降機構位置制御現在値
    bool position_control_lift;       // 昇降機構位置制御モードか否か
    bool position_control_upper_hand; // 上ハンド機構位置制御モードか否か
    bool position_control_under_hand; // 下ハンド機構位置制御モードか否か
} __attribute__((__packed__));

struct operation_data_t
{
    float vx;                     // x軸方向の速度[m/s]
    float vy;                     // y軸方向の速度[m/s]
    float omega;                  // 回転速度[rad/s]
    float width;                  // 上アーム開閉幅
    float depth;                  // 上アーム出し入れ
    float lift;                   // 昇降
    float right_slide;            // 右下ハンド前後スライド
    float right_raise;            // 右下ハンド上下回転
    float left_slide;             // 左下ハンド前後スライド
    float left_raise;             // 左下ハンド上下回転
    uint8_t communication_status; // 通信状態 (0:なし 1:PC-main間通信 2:WiFi 3:BLE)
    uint8_t control_mode;         // 制御モード (0:manual 1:manual(PC) 2:auto(PC))
} __attribute__((__packed__));

struct controller_data_t
{
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
    uint8_t id;        // MDのID
    bool reinitialize; // 再初期化するか否か
    float p;           // 比例ゲイン
    float i;           // 積分ゲイン
    float d;           // 微分ゲイン
} __attribute__((__packed__));
