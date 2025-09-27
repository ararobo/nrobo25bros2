#include "ararobo_robot/controller_node.hpp"

ControllerNode::ControllerNode()
    : rclcpp::Node("controller_node")
{
    pub_joy_ = this->create_publisher<sensor_msgs::msg::Joy>("/controller", 10);
    controller_udp = std::make_shared<SimpleUDP>();
    mainboard_udp = std::make_shared<SimpleUDP>();
    if (!controller_udp->initSocket())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize controller UDP socket");
        return;
    }
    if (!controller_udp->bindSocket(ethernet_config::pc::ip_wifi,
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
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
                                     std::bind(&ControllerNode::timer_callback, this));
}

ControllerNode::~ControllerNode()
{
    controller_udp->closeSocket();
    mainboard_udp->closeSocket();
    RCLCPP_INFO(this->get_logger(), "ControllerNode destroyed");
}

void ControllerNode::timer_callback()
{
    if (controller_udp->recvPacket(controller_union.code, sizeof(controller_data_union_t)))
    {
        is_controller_connected = true;
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
        disconnect_count = 0;
    }
    else
    {
        if (is_controller_connected)
        {
            disconnect_count++;
            if (disconnect_count >= disconnect_threshold)
            {
                is_controller_connected = false;
                disconnect_count = 0;
                RCLCPP_WARN(this->get_logger(), "Controller disconnected");
            }
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
    }
}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerNode>());
    rclcpp::shutdown();
    return 0;
}
