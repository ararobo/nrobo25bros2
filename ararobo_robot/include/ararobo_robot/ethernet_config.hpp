#pragma once
#include <stdint.h>

namespace ethernet_config
{
    namespace main_board
    {
        uint8_t ip[4] = {192, 168, 1, 10}; // メインボードのIPアドレス

        uint16_t port_operation = 5000; // 操作用ポート番号
        uint16_t port_feedback = 5001;  // フィードバック用ポート番号
    }

    namespace pc
    {
        uint8_t ip[4] = {192, 168, 1, 100}; // PCのIPアドレス
        uint16_t port_operation = 5000;     // PCの操作用ポート番号
        uint16_t port_feedback = 5001;      // PCのフィードバック用ポート番号
    }
}