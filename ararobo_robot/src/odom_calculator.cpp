#include "ararobo_robot/odom_calculator.hpp"
#include <cmath> // M_PI, cos, sin のために必要

OdomCalculator::OdomCalculator(double encoder_resolution, double wheel_radius)
    : encoder_resolution(encoder_resolution), wheel_radius(wheel_radius)
{
    robot_velocity[0] = 0.0; // x軸方向の速度 初期化
    robot_velocity[1] = 0.0; // y軸方向の速度 初期化
    // 初期化
    reset_robot_coord();
}

// period_s (秒単位) を引数に追加し、真の速度 (m/s) を計算
void OdomCalculator::set_encoder_count(int16_t vx_count, int16_t vy_count, double period_s)
{
    // エンコーダカウントから、この周期で移動した距離を計算
    // vx_count, vy_count が符号付きのデルタカウントであると仮定
    double distance_x_per_period = (double)vx_count * (2.0 * M_PI * wheel_radius / encoder_resolution);
    double distance_y_per_period = (double)vy_count * (2.0 * M_PI * wheel_radius / encoder_resolution);

    // 速度 (m/s) を計算
    if (period_s > 0.0)
    {
        robot_velocity[0] = distance_x_per_period / period_s; // x軸方向の速度 (m/s)
        robot_velocity[1] = distance_y_per_period / period_s; // y軸方向の速度 (m/s)
    }
    else
    {
        robot_velocity[0] = 0.0;
        robot_velocity[1] = 0.0;
    }
}

// ロボットの座標はFeedbackNodeでIMUと統合して計算するため、この関数は削除
// void OdomCalculator::get_robot_coord(double *x, double *y, double theta, double period)
// {
//     // robot_velocity は既にロボット座標系での速度 (m/s)
//     double vx_robot_frame = robot_velocity[0];
//     double vy_robot_frame = robot_velocity[1];

//     // ロボット座標系での速度をフィールド座標系に変換
//     // 標準的な2D回転行列: [ cos(theta) -sin(theta) ]
//     //                   [ sin(theta)  cos(theta) ]
//     double vx_field_frame = cos(theta) * vx_robot_frame - sin(theta) * vy_robot_frame;
//     double vy_field_frame = sin(theta) * vx_robot_frame + cos(theta) * vy_robot_frame;

//     // フィールド座標系での速度に時間間隔を掛けて距離を出し、座標を更新
//     robot_coord[0] += vx_field_frame * period; // x座標を更新
//     robot_coord[1] += vy_field_frame * period; // y座標を更新

//     *x = robot_coord[0]; // x座標を返す
//     *y = robot_coord[1]; // y座標を返す
// }

void OdomCalculator::reset_robot_coord()
{
    robot_coord[0] = 0.0; // x座標をリセット
    robot_coord[1] = 0.0; // y座標をリセット
}