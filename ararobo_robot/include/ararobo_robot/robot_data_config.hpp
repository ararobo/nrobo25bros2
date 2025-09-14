#pragma once
#include <stdint.h>

struct feedback_data_t
{
    float q_x;                 // ロボットの回転クォータニオン
    float q_y;                 // ロボットの回転クォータニオン
    float q_z;                 // ロボットの回転クォータニオン
    float q_w;                 // ロボットの回転クォータニオン
    int16_t encoder_x;         // x軸方向のエンコーダ値[CRP]
    int16_t encoder_y;         // y軸方向のエンコーダ値[CRP]
    uint16_t distance_base_rf; // 右前の距離センサ[mm]
    uint16_t distance_base_rm; // 右中央の距離センサ[mm]
    uint16_t distance_base_rb; // 右後の距離センサ[mm]
    uint16_t distance_base_lf; // 左前の距離センサ[mm]
    uint16_t distance_base_lm; // 左中央の距離センサ[mm]
    uint16_t distance_base_lb; // 左後の距離センサ[mm]
} __attribute__((__packed__));

struct operation_data_t
{
    float vx;          // x軸方向の速度[m/s]
    float vy;          // y軸方向の速度[m/s]
    float omega;       // 回転速度[rad/s]
    float width;       // 上アーム開閉幅
    float depth;       // 上アーム出し入れ
    float lift;        // 昇降
    float right_slide; // 右下ハンド前後スライド
    float right_raise; // 右下ハンド上下回転
    float left_slide;  // 左下ハンド前後スライド
    float left_raise;  // 左下ハンド上下回転
    uint8_t task_kind; // 動作の種類
} __attribute__((__packed__));

struct controller_data_t
{
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
