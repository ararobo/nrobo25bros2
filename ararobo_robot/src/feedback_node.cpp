#include "ararobo_robot/feedback_node.hpp"

FeedbackNode::FeedbackNode()
    : Node("feedback_node")
{
    // 機体固有値の初期化
    encoder_resolution = 4096.0f; // エンコーダの分解能 (CPR)
    wheel_radius = 0.03f;         // 車輪の半径 (m)
    period_odom = 20;             // オドメトリ計算周期 (ms)

    // UDP通信の初期化
    udp = std::make_shared<SimpleUDP>();
    // OdomCalculatorの初期化
    odom_calculator = std::make_shared<OdomCalculator>(encoder_resolution, wheel_radius);

    if (!udp->initSocket())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize UDP socket");
        return;
    }

    // 受信ポートの設定
    if (!udp->bindSocket(ethernet_config::pc::ip, ethernet_config::main_board::port_feedback))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to bind UDP socket");
        return;
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
        RCLCPP_ERROR(this->get_logger(), "Failed to receive UDP packet");
        return;
    }

    theta = feedback_union.data.yaw; // ヨー角[rad]

    // オドメトリ計算
    odom_calculator->set_encoder_count(feedback_union.data.encoder_x, feedback_union.data.encoder_y);
    odom_calculator->get_robot_coord(&x, &y, theta, period_odom / 1000.0);

    rclcpp::Time now = this->now(); // タイムスタンプを先に取得

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
    odom_msg.twist.twist.linear.x = (double)feedback_union.data.encoder_x / encoder_resolution * M_PI * 2 / (double)period_odom; // x軸方向の速度
    odom_msg.twist.twist.linear.y = (double)feedback_union.data.encoder_y / encoder_resolution * M_PI * 2 / (double)period_odom; // y軸方向の速度
    odom_msg.twist.twist.angular.z = feedback_union.data.yaw_rate;                                                               // z軸方向の角速度
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = 0.0;
    // odom_msg.pose.pose.orientation = tf2::createQuaternionMsgFromYaw(theta);
    odom_msg.pose.pose.orientation = odom_quat_msg; // 変換されたクォータニオンを使用
    pub_odometry_->publish(odom_msg);               // odometryデータの送信

    // 2. TF変換のブロードキャスト
    geometry_msgs::msg::TransformStamped odom_trans;
    odom_trans.header.stamp = now;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    // odom_trans.transform.rotation = tf2::createQuaternionMsgFromYaw(theta);
    odom_trans.transform.rotation = odom_quat_msg; // 同じ変換されたクォータニオンを使用

    tf_broadcaster_->sendTransform(odom_trans);

    RCLCPP_INFO(this->get_logger(), "Odometry: x: %f, y: %f, theta: %f", x, y, theta);
}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FeedbackNode>());
    rclcpp::shutdown();
    return 0;
}