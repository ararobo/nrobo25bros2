#include "ararobo_robot/controller_node.hpp"

ControllerNode::ControllerNode()
    : rclcpp::Node("controller_node")
{
    pub_joy_ = this->create_publisher<sensor_msgs::msg::Joy>("/joy", 10);
    pub_connection_status_ = this->create_publisher<std_msgs::msg::UInt8>("/controller/connection_status", 10);
    controller_udp[0] = std::make_shared<SimpleUDP>();
    controller_udp[1] = std::make_shared<SimpleUDP>();
    mainboard_udp = std::make_shared<SimpleUDP>();
    if (!controller_udp[0]->initSocket())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize controller UDP socket");
        return;
    }
    if (!controller_udp[0]->bindSocket(ethernet_config::pc::ip_wifi_1,
                                       ethernet_config::controller::port_controller))
    {
        RCLCPP_ERROR(this->get_logger(), "bind error\n");
    }

    if (!controller_udp[1]->initSocket())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize controller UDP socket");
        return;
    }
    if (!controller_udp[1]->bindSocket(ethernet_config::pc::ip_wifi_2,
                                       ethernet_config::controller::port_controller))
    {
        RCLCPP_ERROR(this->get_logger(), "bind error\n");
    }

    if (!mainboard_udp->initSocket())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize mainboard UDP socket");
        return;
    }
    if (!mainboard_udp->bindSocket(ethernet_config::pc::ip,
                                   ethernet_config::main_board::port_controller_ble))
    {
        RCLCPP_ERROR(this->get_logger(), "bind error\n");
    }
    recv_timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
                                          std::bind(&ControllerNode::recv_timer_callback, this));
    ping_timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
                                          std::bind(&ControllerNode::ping_timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "ControllerNode started");
}

ControllerNode::~ControllerNode()
{
    controller_udp[0]->closeSocket();
    controller_udp[1]->closeSocket();
    mainboard_udp->closeSocket();
    RCLCPP_INFO(this->get_logger(), "ControllerNode destroyed");
}

void ControllerNode::recv_timer_callback()
{
    if (controller_udp[0]->recvPacket(controller_union.code, sizeof(controller_data_union_t)) && avg_ping_time < ping_time_threshold)
    {
        sensor_msgs::msg::Joy joy_msg;
        joy_msg.header.stamp = this->now();
        joy_msg.axes.resize(4);
        joy_msg.buttons.resize(8);
        joy_msg.axes[0] = controller_union.data.left_stick_x / 127.f;
        joy_msg.axes[1] = controller_union.data.left_stick_y / 127.f;
        joy_msg.axes[2] = controller_union.data.right_stick_x / 127.f;
        joy_msg.axes[3] = controller_union.data.right_stick_y / 127.f;
        joy_msg.buttons[0] = controller_union.data.buttons.l_up;
        joy_msg.buttons[1] = controller_union.data.buttons.l_down;
        joy_msg.buttons[2] = controller_union.data.buttons.l_left;
        joy_msg.buttons[3] = controller_union.data.buttons.l_right;
        joy_msg.buttons[4] = controller_union.data.buttons.r_up;
        joy_msg.buttons[5] = controller_union.data.buttons.r_down;
        joy_msg.buttons[6] = controller_union.data.buttons.r_left;
        joy_msg.buttons[7] = controller_union.data.buttons.r_right;
        pub_joy_->publish(joy_msg);
        controller_disconnect_count = 0;
    }
    else
    {
        if (controller_udp[1]->recvPacket(controller_union.code, sizeof(controller_data_union_t)) && avg_ping_time < ping_time_threshold)
        {
            sensor_msgs::msg::Joy joy_msg;
            joy_msg.header.stamp = this->now();
            joy_msg.axes.resize(4);
            joy_msg.buttons.resize(8);
            joy_msg.axes[0] = controller_union.data.left_stick_x / 127.f;
            joy_msg.axes[1] = controller_union.data.left_stick_y / 127.f;
            joy_msg.axes[2] = controller_union.data.right_stick_x / 127.f;
            joy_msg.axes[3] = controller_union.data.right_stick_y / 127.f;
            joy_msg.buttons[0] = controller_union.data.buttons.l_up;
            joy_msg.buttons[1] = controller_union.data.buttons.l_down;
            joy_msg.buttons[2] = controller_union.data.buttons.l_left;
            joy_msg.buttons[3] = controller_union.data.buttons.l_right;
            joy_msg.buttons[4] = controller_union.data.buttons.r_up;
            joy_msg.buttons[5] = controller_union.data.buttons.r_down;
            joy_msg.buttons[6] = controller_union.data.buttons.r_left;
            joy_msg.buttons[7] = controller_union.data.buttons.r_right;
            pub_joy_->publish(joy_msg);
            controller_disconnect_count = 0;
        }
        if (is_controller_connected)
        {
            controller_disconnect_count++;
        }
        if (controller_disconnect_count >= controller_disconnect_threshold)
        {
            controller_disconnect_count = 0;
            is_controller_connected = false;
            auto connection_msg = std_msgs::msg::UInt8();
            connection_msg.data = 1; // 1: 無線通信なし
            pub_connection_status_->publish(connection_msg);
            RCLCPP_WARN(this->get_logger(), "Controller disconnected");
        }
    }

    if (!is_controller_connected && mainboard_udp->recvPacket(controller_union.code, sizeof(controller_data_union_t)))
    {

        sensor_msgs::msg::Joy joy_msg;
        joy_msg.header.stamp = this->now();
        joy_msg.axes.resize(4);
        joy_msg.buttons.resize(8);
        joy_msg.axes[0] = controller_union.data.left_stick_x / 127.f;
        joy_msg.axes[1] = controller_union.data.left_stick_y / 127.f;
        joy_msg.axes[2] = controller_union.data.right_stick_x / 127.f;
        joy_msg.axes[3] = controller_union.data.right_stick_y / 127.f;
        joy_msg.buttons[0] = controller_union.data.buttons.l_up;
        joy_msg.buttons[1] = controller_union.data.buttons.l_down;
        joy_msg.buttons[2] = controller_union.data.buttons.l_left;
        joy_msg.buttons[3] = controller_union.data.buttons.l_right;
        joy_msg.buttons[4] = controller_union.data.buttons.r_up;
        joy_msg.buttons[5] = controller_union.data.buttons.r_down;
        joy_msg.buttons[6] = controller_union.data.buttons.r_left;
        joy_msg.buttons[7] = controller_union.data.buttons.r_right;
        pub_joy_->publish(joy_msg);
        mainboard_disconnect_count = 0;
    }
    else
    {
        if (is_mainboard_connected)
        {
            mainboard_disconnect_count++;
        }
        if (mainboard_disconnect_count >= mainboard_disconnect_threshold)
        {
            mainboard_disconnect_count = 0;
            is_mainboard_connected = false;
            auto connection_msg = std_msgs::msg::UInt8();
            connection_msg.data = 1; // 1: 無線通信なし
            pub_connection_status_->publish(connection_msg);
            RCLCPP_WARN(this->get_logger(), "Mainboard disconnected");
        }
    }
}

void ControllerNode::ping_timer_callback()
{
    avg_ping_time = get_ping_time();
    if (avg_ping_time >= ping_time_threshold)
    {
        RCLCPP_WARN(this->get_logger(), "Ping time too high: %.2f ms", avg_ping_time);
    }
    else if (avg_ping_time >= 0)
    {
        RCLCPP_INFO(this->get_logger(), "Ping time: %.2f ms", avg_ping_time);
    }
}

double ControllerNode::get_ping_time()
{
    // ping コマンドの例: 1回だけ実行し、結果の要約（summary）を出力する
    // -c 1: 1回だけ実行
    // -W 1: タイムアウトを1秒に設定
    // {マイコンのIP}: 監視対象のマイコンのIPアドレス
    const std::string PING_COMMAND = "ping -c 1 -W 1 " + target_ip_address;

    FILE *pipe = popen(PING_COMMAND.c_str(), "r");
    if (!pipe)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to run ping command");
        return -1.0;
    }

    char buffer[128];
    std::string result = "";
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr)
    {
        result += buffer;
    }
    pclose(pipe);

    std::regex rtt_regex("rtt min/avg/max/mdev = ([0-9.]+)/([0-9.]+)/([0-9.]+)/([0-9.]+) ms");
    std::smatch match;

    if (std::regex_search(result, match, rtt_regex) && match.size() == 5)
    {
        // match[2] が平均 (avg) のRTT値
        double avg_rtt_ms = std::stod(match[2].str());
        return avg_rtt_ms;
    }
    else
    {
        // RTTの行が見つからない = パケットロスやタイムアウト
        RCLCPP_ERROR(this->get_logger(), "Ping output parsing failed");
        return -1.0;
    }
    RCLCPP_ERROR(this->get_logger(), "Ping failed or timed out");
    return -1.0;
}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerNode>());
    rclcpp::shutdown();
    return 0;
}
