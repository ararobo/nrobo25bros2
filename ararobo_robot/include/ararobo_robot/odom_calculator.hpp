#pragma once
#include <math.h>   // M_PI のために必要
#include <stdint.h> // int16_t のために必要

/**
 * @brief 接地エンコーダーからのオドメトリ計算を行うクラス
 *
 */
class OdomCalculator
{
private:
    /* 機体固有値 */
    double encoder_resolution; // エンコーダの分解能
    double wheel_radius;       // 車輪の半径
    /* 内部使用変数 */
    double robot_coord[2]; // ロボットの座標 (x, y)

    // convert_coord は get_robot_coord の内部に統合するため削除、または private のままにするなら使わない
    // void convert_coord(double *vx, double *vy, double theta);

public:
    // FeedbackNodeからアクセスするために public にするか、getter を追加
    double robot_velocity[2]; // ロボットの速度 (x, y) [m/s] (ロボット座標系)

    /**
     * @brief コンストラクタ
     * @param encoder_resolution エンコーダの分解能[CPR]
     * @param wheel_radius 車輪の半径[m]
     */
    OdomCalculator(double encoder_resolution, double wheel_radius);

    /**
     * @brief エンコーダの値を設定し、ロボットの速度を計算する
     * @param vx_count x軸方向のエンコーダのデルタ値 (この周期でのカウント変化量)
     * @param vy_count y軸方向のエンコーダのデルタ値 (この周期でのカウント変化量)
     * @param period_s デルタ値が計測された時間周期[s]
     */
    void set_encoder_count(int16_t vx_count, int16_t vy_count, double period_s);

    /**
     * @brief ロボットの座標を更新・取得する
     *
     * @param x x座標[m] (出力)
     * @param y y座標[m] (出力)
     * @param theta 機体の角度[rad]
     * @param period 計算周期[s] (get_robot_coordが呼び出された時間間隔)
     * @details ロボットの座標は、ロボットの初期位置からの相対座標である。
     * @note set_encoder_countで計算された速度を基に座標を更新する。
     */
    void get_robot_coord(double *x, double *y, double theta, double period);

    /**
     * @brief ロボットの座標をリセットする
     * @details ロボットの座標をリセットする。ロボットの初期位置からの相対座標を0にする。
     *
     */
    void reset_robot_coord();
};