#include "ararobo_robot/feedback_node.hpp"
#include <cmath> // M_PI のために必要 (thetaから角速度を計算するため)
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // tf2::toMsg のために必要

FeedbackNode::FeedbackNode()
    : Node("feedback_node")
{
    // 機体固有値の初期化
    encoder_resolution = 4096.0; // エンコーダの分解能 (CPR)
    wheel_radius = 0.03;         // 車輪の半径 (m)
    period_odom = 20;            // オドメトリ計算周期 (ms)

    // UDP通信の初期化
    udp = std::make_shared<SimpleUDP>();
    // OdomCalculatorの初期化
    odom_calculator = std::make_shared<OdomCalculator>(encoder_resolution, wheel_radius);

    if (!udp->initSocket())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize UDP socket");
        return; // 初期化失敗で終了
    }

    // 受信ポートの設定
    if (!udp->bindSocket(ethernet_config::pc::ip, ethernet_config::main_board::port_feedback))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to bind UDP socket");
        return; // 初期化失敗で終了
    }

    // Odometryパブリッシャーの初期化
    pub_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "/odom", 10);
    // TF Broadcasterの初期化
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // タイマーの設定
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(period_odom),
        std::bind(&FeedbackNode::timer_callback, this));

    // prev_timeの初期化 (必須)
    prev_time = this->now();
    prev_theta = 0.0; // 初期ヨー角も0に設定

    RCLCPP_INFO(this->get_logger(), "FeedbackNode initialized");
}

FeedbackNode::~FeedbackNode()
{
    udp->closeSocket();
    RCLCPP_INFO(this->get_logger(), "FeedbackNode destroyed");
}

void FeedbackNode::timer_callback()
{
    // UDPパケットの受信
    int recv_size = udp->recvPacket(feedback_union.code, sizeof(feedback_union));
    if (recv_size < 0)
    {
        // エラーログを出すが、タイマーコールバックを続行することで、
        // UDPパケットが一時的に途切れてもシステム全体が停止しないようにする
        RCLCPP_WARN(this->get_logger(), "Failed to receive UDP packet or no data received. Recv size: %d", recv_size);
        // ここでreturnすると、オドメトリやTFが更新されなくなるので注意
        // 代わりに、前回のデータを使用するか、速度を0に設定するなどの対処が必要になる
        // 今回はとりあえずログだけ出して続行するが、場合によってはreturnも考慮
        // if (recv_size == 0) return; // データがない場合は処理しない
        // if (recv_size < 0) return; // エラーの場合は処理しない
    }

    // ヨー角はボードから直接送られてくる値を使用
    // ここで feedback_union.data.yaw が [rad] 単位であることを確認
    theta = feedback_union.data.yaw;

    // オドメトリ計算
    // `OdomCalculator::set_encoder_count` に `period_s` を渡す
    double current_period_s = static_cast<double>(period_odom) / 1000.0;
    odom_calculator->set_encoder_count(feedback_union.data.encoder_x, feedback_union.data.encoder_y, current_period_s);
    // `OdomCalculator::get_robot_coord` に `period_s` を渡す
    odom_calculator->get_robot_coord(&x, &y, theta, current_period_s);

    rclcpp::Time now = this->now(); // 現在のタイムスタンプを取得

    // ヨー角からtf2::Quaternionを作成
    tf2::Quaternion q;
    q.setRPY(0, 0, theta); // ロール、ピッチ、ヨー

    // tf2::Quaternionをgeometry_msgs::msg::Quaternionに変換
    geometry_msgs::msg::Quaternion odom_quat_msg = tf2::toMsg(q);

    // 1. Odometryメッセージの配信
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = now;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    // `odom_calculator`で計算された速度 (m/s) をそのまま使用
    odom_msg.twist.twist.linear.x = odom_calculator->robot_velocity[0]; // ロボット座標系でのx速度 (m/s)
    odom_msg.twist.twist.linear.y = odom_calculator->robot_velocity[1]; // ロボット座標系でのy速度 (m/s)

    // 角速度の計算
    // prev_time が正しく初期化されていることを確認
    double dt = (now.seconds() - prev_time.seconds());
    if (dt > 0.0) // ゼロ除算を避けるため
    {
        // 角度は -PI から PI の範囲に正規化されていることが前提
        double dtheta = theta - prev_theta;
        // 角度が180度以上異なる場合のラップアラウンドを処理 (オプション、必要であれば)
        // if (dtheta > M_PI) dtheta -= 2 * M_PI;
        // if (dtheta < -M_PI) dtheta += 2 * M_PI;
        odom_msg.twist.twist.angular.z = dtheta / dt; // z軸方向の角速度 (rad/s)
    }
    else
    {
        odom_msg.twist.twist.angular.z = 0.0;
    }

    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = odom_quat_msg;
    pub_odometry_->publish(odom_msg); // odometryデータの送信

    // 2. TF変換のブロードキャスト
    geometry_msgs::msg::TransformStamped odom_trans;
    odom_trans.header.stamp = now;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat_msg; // 同じ変換されたクォータニオンを使用

    tf_broadcaster_->sendTransform(odom_trans);

    prev_time = now;    // 前回のタイムスタンプを更新
    prev_theta = theta; // 前回のヨー角を更新

    // ログ出力 (より詳細に速度も出力)
    RCLCPP_INFO(this->get_logger(), "Odometry: x: %f, y: %f, theta: %f, linear_vx: %f, linear_vy: %f, angular_wz: %f",
                x, y, theta, odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y, odom_msg.twist.twist.angular.z);
}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FeedbackNode>());
    rclcpp::shutdown();
    return 0;
}