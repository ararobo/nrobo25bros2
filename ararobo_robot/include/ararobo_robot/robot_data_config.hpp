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
    float vx;    // x軸方向の速度[m/s]
    float vy;    // y軸方向の速度[m/s]
    float omega; // 回転速度[rad/s]
    float ur;    // 上アーム右位置[m]
    float ul;    // 上アーム左位置[m]
    float lr;    // 下アーム右位置[m]
    float ll;    // 下アーム左位置[m]
    float ur_w;  // 右上アーム幅[m]
    float ul_w;  // 左上アーム幅[m]
    float lr_w;  // 右上アーム幅[m]
    float ll_w;  // 左上アーム幅[m]
    float lift;  // 昇降
} __attribute__((__packed__));