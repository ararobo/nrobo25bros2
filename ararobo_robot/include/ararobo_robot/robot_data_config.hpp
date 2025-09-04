#pragma once
#include <stdint.h>

struct feedback_data_t
{
    float yaw;         // ヨー角[rad]
    int16_t encoder_x; // x軸方向のエンコーダ値
    int16_t encoder_y; // y軸方向のエンコーダ値
    uint16_t distance_base_rf;
    uint16_t distance_base_rm;
    uint16_t distance_base_rb;
    uint16_t distance_base_lf;
    uint16_t distance_base_lm;
    uint16_t distance_base_lb;
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
} __attribute__((__packed__));