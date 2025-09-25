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
    // 操作用ポートの設定
    udp->setTxAddr(ethernet_config::main_board::ip,
                   ethernet_config::main_board::port_operation);
    sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        std::bind(&OperationNode::cmd_vel_callback, this, std::placeholders::_1));
    lift_vel_ = this->create_subscription<std_msgs::msg::Float32>(
        "/lift/target", 10,
        std::bind(&OperationNode::lift_vel_callback, this, std::placeholders::_1));

    sub_upper_hand_width_ = this->create_subscription<std_msgs::msg::Float32>(
        "/hand/upper/width", 10,
        [&](const std_msgs::msg::Float32::SharedPtr msg) -> void
        { operation_data.width = msg->data; });

    sub_upper_hand_depth_ = this->create_subscription<std_msgs::msg::Float32>(
        "/hand/upper/depth", 10,
        [&](const std_msgs::msg::Float32::SharedPtr msg) -> void
        { operation_data.depth = msg->data; });

    sub_under_hand_slide_ = this->create_subscription<std_msgs::msg::Float32>(
        "/hand/under/slide", 10,
        [&](const std_msgs::msg::Float32::SharedPtr msg) -> void
        {   operation_data.left_slide = -msg->data;
            operation_data.right_slide = msg->data; });

    sub_under_hand_raise_ = this->create_subscription<std_msgs::msg::Float32>(
        "/hand/under/raise", 10,
        [&](const std_msgs::msg::Float32::SharedPtr msg) -> void
        { operation_data.left_raise = -msg->data;
        operation_data.right_raise = msg->data; });
    sub_communication_status_ = this->create_subscription<std_msgs::msg::UInt8>(
        "/communication_status", 10,
        [&](const std_msgs::msg::UInt8::SharedPtr msg) -> void
        { operation_data.comunication_status = msg->data; });

    timer_ = this->create_wall_timer(std::chrono::milliseconds(15),
                                     std::bind(&OperationNode::timer_callback, this));

    operation_data.comunication_status = 1; // PC-main間通信
    RCLCPP_INFO(this->get_logger(), "OperationNode started");
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

void OperationNode::lift_vel_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    operation_data.lift = msg->data; // リフトの速度[m/s]
    RCLCPP_INFO(this->get_logger(), "lift_vel: %f", msg->data);
}

void OperationNode::timer_callback()
{
    operation_union.data = operation_data; // データをユニオンにコピー
    if (!udp->sendPacket(operation_union.code, sizeof(operation_union)))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to send operation data via UDP");
    }
}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OperationNode>());
    rclcpp::shutdown();
    return 0;
}
