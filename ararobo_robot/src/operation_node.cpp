#include "ararobo_robot/operation_node.hpp"

OperationNode::OperationNode()
    : Node("operation_node")
{
    // UDP通信の初期化
    udp = std::make_shared<SimpleUDP>();
    if (!udp->initSocket())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize UDP socket");
        return;
    }
    sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/robot/move", 10,
        std::bind(&OperationNode::cmd_vel_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
                                     std::bind(&OperationNode::timer_callback, this));
}

OperationNode::~OperationNode()
{
    udp->closeSocket();
    RCLCPP_INFO(this->get_logger(), "OperationNode destroyed");
}

void OperationNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    operation_data.vx = msg->linear.x;     // x軸方向の速度[m/s]
    operation_data.vy = msg->linear.y;     // y軸方向の速度[m/s]
    operation_data.omega = msg->angular.z; // 回転速度[rad/s]
    RCLCPP_INFO(this->get_logger(), "cmd_vel: linear.x: %f, linear.y: %f, angular.z: %f",
                msg->linear.x, msg->linear.y, msg->angular.z);
}

void OperationNode::timer_callback()
{
    operation_union.data = operation_data; // データをユニオンにコピー
    if (!udp->sendPacket(operation_union.code, sizeof(operation_union)))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to send operation data via UDP");
    }
}