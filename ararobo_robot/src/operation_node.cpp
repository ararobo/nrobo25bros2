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
    sub_md_data_ = this->create_subscription<ararobo_msgs::msg::MdData>(
        "/md_data", 10,
        std::bind(&OperationNode::md_data_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(std::chrono::milliseconds(20),
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

void OperationNode::md_data_callback(const ararobo_msgs::msg::MdData::SharedPtr msg)
{
    operation_data.ur = msg->ur;     // 上アーム右位置[m]
    operation_data.ul = msg->ul;     // 上アーム左位置[m]
    operation_data.lr = msg->lr;     // 下アーム右位置[m]
    operation_data.ll = msg->ll;     // 下アーム左位置[m]
    operation_data.ur_w = msg->ur_w; // 右上アーム幅[m]
    operation_data.ul_w = msg->ul_w; // 左上アーム幅[m]
    operation_data.lr_w = msg->lr_w; // 右下アーム幅[m]
    operation_data.ll_w = msg->ll_w; // 左下アーム幅[m]
    RCLCPP_INFO(this->get_logger(), "MDData: ur: %f, ul: %f, lr: %f, ll: %f, ur_w: %f, ul_w: %f, lr_w: %f, ll_w: %f",
                msg->ur, msg->ul, msg->lr, msg->ll,
                msg->ur_w, msg->ul_w, msg->lr_w, msg->ll_w);
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
