#include "ararobo_robot/odom_calculator.hpp"
#include <cmath>

OdomCalculator::OdomCalculator(double encoder_resolution, double wheel_radius)
    : encoder_resolution(encoder_resolution), wheel_radius(wheel_radius)
{
    // メンバ変数の初期化
    robot_velocity[0] = 0.0;
    robot_velocity[1] = 0.0;
    robot_angular_velocity = 0.0;
    prev_theta = 0.0;
    robot_coord[0] = 0.0;
    robot_coord[1] = 0.0;
}

void OdomCalculator::update_odom(int16_t vx_count, int16_t vy_count, double current_theta, double dt)
{
    // ゼロ除算を避けるためのチェック
    if (dt <= 0.0) {
        robot_velocity[0] = 0.0;
        robot_velocity[1] = 0.0;
        robot_angular_velocity = 0.0;
        return;
    }

    // エンコーダカウントから、この周期で移動した距離を計算
    double distance_x_per_period = static_cast<double>(vx_count) * (2.0 * M_PI * wheel_radius / encoder_resolution);
    double distance_y_per_period = static_cast<double>(vy_count) * (2.0 * M_PI * wheel_radius / encoder_resolution);

    // ロボット座標系での速度 (m/s) を計算
    robot_velocity[0] = distance_x_per_period / dt;
    robot_velocity[1] = distance_y_per_period / dt;

    // ロボットの向きの変化量を計算し、-piからpiの範囲に正規化
    double dtheta = current_theta - prev_theta;
    while (dtheta > M_PI) dtheta -= 2.0 * M_PI;
    while (dtheta < -M_PI) dtheta += 2.0 * M_PI;
    
    // オドメトリ更新に用いる平均ヨー角を計算
    double theta_avg = prev_theta + dtheta / 2.0;
    
    // ロボット座標系での速度をフィールド座標系に変換
    double vx_field_frame = cos(theta_avg) * robot_velocity[0] - sin(theta_avg) * robot_velocity[1];
    double vy_field_frame = sin(theta_avg) * robot_velocity[0] + cos(theta_avg) * robot_velocity[1];

    // フィールド座標系での位置を更新（積分）
    robot_coord[0] += vx_field_frame * dt;
    robot_coord[1] += vy_field_frame * dt;

    // 角速度を計算
    robot_angular_velocity = dtheta / dt;

    // 次回計算のために現在のthetaを保存
    prev_theta = current_theta;
}

void OdomCalculator::reset_robot_coord()
{
    robot_coord[0] = 0.0;
    robot_coord[1] = 0.0;
}