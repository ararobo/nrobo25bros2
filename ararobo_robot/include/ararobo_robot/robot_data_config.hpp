#pragma once
#include <stdint.h>

struct feedback_data_t
{
    float yaw;         // ヨー角[rad]
    float pitch;       // ピッチ角[rad]
    float roll;        // ロール角[rad]
    int16_t encoder_x; // x軸方向のエンコーダ値
    int16_t encoder_y; // y軸方向のエンコーダ値
} __attribute__((__packed__));

struct operation_data_t
{
    float vx;    // x軸方向の速度[m/s]
    float vy;    // y軸方向の速度[m/s]
    float omega; // 回転速度[rad/s]
} __attribute__((__packed__));