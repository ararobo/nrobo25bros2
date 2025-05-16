#include "ararobo_can/odom_node.hpp"

OdomNode::OdomNode()
    : Node("odom_node")
{
    // CANドライバの初期化
    pc_data_slave_ = std::make_shared<PCDataSlave>();
    pc_data_slave_->init();

    pub_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "/odom", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&OdomNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "OdomNode initialized");
}

void OdomNode::timer_callback()
{
    pc_data_slave_->can_receive_process(); // CAN受信処理
    // odometryデータの取得
    if (pc_data_slave_->get_odometry(&x, &y, &theta))
    {
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = this->now();
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";
        odom_msg.pose.pose.position.x = x;
        odom_msg.pose.pose.position.y = y;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation.z = theta;
        pub_odometry_->publish(odom_msg); // odometryデータの送信
        RCLCPP_INFO(this->get_logger(), "Odometry: x: %f, y: %f, theta: %f",
                    x, y, theta);
    }
}

OdomNode::~OdomNode()
{
    // CANドライバの終了処理
    pc_data_slave_->~PCDataSlave();
    RCLCPP_INFO(this->get_logger(), "OdomNode destroyed");
}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomNode>());
    rclcpp::shutdown();
    return 0;
}