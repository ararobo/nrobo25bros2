#include "ararobo_robot/feedback_node.hpp"
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.h>

FeedbackNode::FeedbackNode()
    : Node("feedback_node")
{
    // 機体固有値の初期化
    const double encoder_resolution = 4096.0; // エンコーダの分解能 (CPR)
    const double wheel_radius = 0.03;         // 車輪の半径 (m)
    const int period_odom = 20;            // オドメトリ計算周期 (ms)

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
    pub_hand_width_ = this->create_publisher<std_msgs::msg::Float32>(
        "current_width", 10);
    pub_hand_depth_ = this->create_publisher<std_msgs::msg::Float32>(
        "current_depth", 10);
    pub_width_distance_ = this->create_publisher<std_msgs::msg::Float32>(
        "width_distance", 10);
    pub_depth_distance_ = this->create_publisher<std_msgs::msg::Float32>(
        "depth_distance", 10);
    // TF Broadcasterの初期化
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // タイマーの設定
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(period_odom),
        std::bind(&FeedbackNode::timer_callback, this));
    
    // prev_timeの初期化
    prev_time = this->now();
    
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
    if (recv_size != sizeof(feedback_union))
    {
        if (recv_size < 0) {
            RCLCPP_WARN(this->get_logger(), "Failed to receive UDP packet. Recv size: %d", recv_size);
        } else {
            RCLCPP_WARN(this->get_logger(), "Received packet size mismatch. Expected: %zu, Received: %d", sizeof(feedback_union), recv_size);
        }
        return;
    }

    tf2::Quaternion q_tf2;
    q_tf2.setX(feedback_union.data.q_x);
    q_tf2.setY(feedback_union.data.q_y);
    q_tf2.setZ(feedback_union.data.q_z);
    q_tf2.setW(feedback_union.data.q_w);

    tf2::Matrix3x3 matrix(q_tf2);
    double roll, pitch, yaw;
    matrix.getRPY(roll, pitch, yaw);
    double theta = yaw;

    // 現在時刻を取得し、時間差分を計算
    rclcpp::Time current_time = this->now();
    double dt = (current_time - prev_time).seconds();
    prev_time = current_time;

    // オドメトリ計算をOdomCalculatorに委譲
    odom_calculator->update_odom(feedback_union.data.encoder_x, -feedback_union.data.encoder_y, theta, dt);

    // tf2::Quaternionをgeometry_msgs::msg::Quaternionに変換
    geometry_msgs::msg::Quaternion odom_quat_msg = tf2::toMsg(q_tf2);

    // odom
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    odom_msg.twist.twist.linear.x = odom_calculator->robot_velocity[0];
    odom_msg.twist.twist.linear.y = odom_calculator->robot_velocity[1];
    odom_msg.twist.twist.angular.z = odom_calculator->robot_angular_velocity;

    odom_msg.pose.pose.position.x = odom_calculator->robot_coord[0];
    odom_msg.pose.pose.position.y = odom_calculator->robot_coord[1];
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = odom_quat_msg;
    pub_odometry_->publish(odom_msg);

    // tf2
    geometry_msgs::msg::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = odom_calculator->robot_coord[0];
    odom_trans.transform.translation.y = odom_calculator->robot_coord[1];
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat_msg;

    tf_broadcaster_->sendTransform(odom_trans);
}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FeedbackNode>());
    rclcpp::shutdown();
    return 0;
}