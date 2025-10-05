#include "ararobo_robot/param_node.hpp"

ParamNode::ParamNode()
    : Node("param_node")
{
    // UDP通信の初期化
    udp = std::make_shared<SimpleUDP>();

    if (!udp->initSocket())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize UDP socket");
        return; // 初期化失敗で終了
    }
    udp->setTxAddr(ethernet_config::main_board::ip,
                   ethernet_config::main_board::port_pid_gain);
    // PIDゲインサブスクライバーの初期化
    sub_pid_gain_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/pid_gain", 10,
        std::bind(&ParamNode::pid_gain_callback, this, std::placeholders::_1));
}

ParamNode::~ParamNode()
{
    udp->closeSocket();
}

void ParamNode::pid_gain_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (msg->data.size() != 4)
    {
        RCLCPP_WARN(this->get_logger(), "Received PID gain array is invalid size: %zu", msg->data.size());
        return;
    }

    motor_id = int(msg->data[0]);
    if (motor_id < 0 || motor_id > 11)
    {
        RCLCPP_WARN(this->get_logger(), "Received motor ID is out of range: %d", motor_id);
        return;
    }

    // 受信データをPIDゲイン構造体にコピー
    pid_gain_union.data.id = motor_id;        // モーターIDを設定
    pid_gain_union.data.reinitialize = false; // 再初期化フラグを設定
    pid_gain_union.data.p = msg->data[1];
    pid_gain_union.data.i = msg->data[2];
    pid_gain_union.data.d = msg->data[3];

    // UDPで送信
    if (!udp->sendPacket(pid_gain_union.code, sizeof(pid_gain_t)))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to send PID gain data via UDP");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Sent PID gain data: id=%d, P=%.2f, I=%.2f, D=%.2f",
                    pid_gain_union.data.id,
                    pid_gain_union.data.p,
                    pid_gain_union.data.i,
                    pid_gain_union.data.d);
    }
}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParamNode>());
    rclcpp::shutdown();
    return 0;
}
