#pragma once
#include <stdint.h>

union led_config_t
{
    struct
    {
        uint8_t signal;    // 通信状態 (0:なし 1:PC-main間通信 2:BLE 3:WiFi-1 4:WiFi-2)
        bool ethernet;     // false:not_error    true:error
        bool operate;      // false:auto         true:manual
        bool speed;        // false:low          true:high
        bool acceleration; // false:low          true:high
        bool ToF;          // false:not_success  true:success
    } __attribute__((__packed__));
    uint8_t code[6];
} __attribute__((__packed__));