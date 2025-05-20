#include "ararobo_can/odom_node.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // クォータニオン生成のためのヘッダー
#include <tf2_ros/transform_broadcaster.h> // ROS2用のTFブロードキャスターヘッダー

std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

OdomNode::OdomNode()
    : Node("odom_node")
{
    // CANドライバの初期化
    pc_data_slave_ = std::make_shared<PCDataSlave>();
    pc_data_slave_->init();

    pub_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "/odom", 10);

    // TF Broadcasterの初期化
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&OdomNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "OdomNode initialized");
}

void OdomNode::timer_callback()
{
    pc_data_slave_->can_receive_process(); // CAN受信処理
    // odometryデータの取得
    if (pc_data_slave_->get_odometry(&x, &y, &theta))
    {
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
        odom_msg.pose.pose.position.x = x;
        odom_msg.pose.pose.position.y = y;
        odom_msg.pose.pose.position.z = 0.0;
        //odom_msg.pose.pose.orientation = tf2::createQuaternionMsgFromYaw(theta);
        odom_msg.pose.pose.orientation = odom_quat_msg; // 変換されたクォータニオンを使用
        pub_odometry_->publish(odom_msg); // odometryデータの送信

        // 2. TF変換のブロードキャスト
        geometry_msgs::msg::TransformStamped odom_trans;
        odom_trans.header.stamp = now;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        //odom_trans.transform.rotation = tf2::createQuaternionMsgFromYaw(theta);
        odom_trans.transform.rotation = odom_quat_msg; // 同じ変換されたクォータニオンを使用

        tf_broadcaster_->sendTransform(odom_trans);

        RCLCPP_INFO(this->get_logger(), "Odometry: x: %f, y: %f, theta: %f",
                    x, y, theta);
    }
}

OdomNode::~OdomNode()
{
    // CANドライバの終了処理
    //pc_data_slave_->~PCDataSlave();
    RCLCPP_INFO(this->get_logger(), "OdomNode destroyed");
}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomNode>());
    rclcpp::shutdown();
    return 0;
}