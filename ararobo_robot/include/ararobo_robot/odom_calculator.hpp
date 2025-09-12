#pragma once
#include <math.h>   // M_PIのために必要
#include <stdint.h> // int16_tのために必要

/**
 * @brief 接地エンコーダーとIMUからのオドメトリを計算するクラス
 */
class OdomCalculator
{
public:
    /**
     * @brief コンストラクタ
     * @param encoder_resolution エンコーダの分解能[CPR]
     * @param wheel_radius 車輪の半径[m]
     */
    OdomCalculator(double encoder_resolution, double wheel_radius);

    /**
     * @brief ロボットのオドメトリ（位置・速度・角速度）を更新する
     * @param vx_count x軸方向のエンコーダのデルタ値
     * @param vy_count y軸方向のエンコーダのデルタ値
     * @param current_theta 現在の機体のヨー角[rad]
     * @param dt 前回の更新からの経過時間[s]
     */
    void update_odom(int16_t vx_count, int16_t vy_count, double current_theta, double dt);

    /**
     * @brief ロボットの座標をリセットする
     */
    void reset_robot_coord();

    // FeedbackNodeからアクセスするためのパブリックメンバ
    double robot_coord[2];         // ロボットの座標 (x, y) [m] (フィールド座標系)
    double robot_velocity[2];      // ロボットの速度 (x, y) [m/s] (ロボット座標系)
    double robot_angular_velocity; // ロボットの角速度 [rad/s]

private:
    /* 機体固有値 */
    double encoder_resolution; // エンコーダの分解能
    double wheel_radius;       // 車輪の半径

    /* 内部使用変数 */
    double prev_theta; // 前回の更新時のヨー角
};