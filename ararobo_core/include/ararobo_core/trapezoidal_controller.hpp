#pragma once
#include <stdint.h>

template <typename T>
class TrapezoidalController
{
private:
    T prev_out = 0;             // 前回の出力
    uint8_t control_cycle = 10; // 制御周期

public:
    TrapezoidalController()
    {
        reset();
    }
    /**
     * @brief 制御周期を設定する
     * @param cycle 制御周期[ms]
     */
    void set_control_cycle(uint8_t cycle)
    {
        control_cycle = cycle;
    }

    /**
     * @brief 台形制御を行う
     *
     * @param output 出力
     * @param max_acceleration 最大加速度
     * @return T 台形制御の出力
     */
    T trapezoidal_control(T output, T max_acceleration)
    {
        // 台形制御の計算
        if (output > prev_out + max_acceleration * control_cycle)
        {
            output = prev_out + max_acceleration * control_cycle;
        }
        else if (output < prev_out - max_acceleration * control_cycle)
        {
            output = prev_out - max_acceleration * control_cycle;
        }
        prev_out = output;

        return output;
    }

    /**
     * @brief 台形制御の初期化
     *
     */
    void reset()
    {
        prev_out = 0;
    }
};