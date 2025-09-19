#include "ararobo_core/core_node.hpp"
#include "ararobo_core/ethernet_config.hpp"

CoreNode::CoreNode() : Node("core_node")
{
    udp = std::make_shared<SimpleUDP>();
    sub_phone_cmd_vel = this->create_subscription<geometry_msgs::msg::Twist>(
        "/phone/cmd_vel", 10, std::bind(&CoreNode::cmd_vel_callback, this, std::placeholders::_1));
    sub_phone_mode = this->create_subscription<std_msgs::msg::UInt8>(
        "/phone/mode", 10, [&](const std_msgs::msg::UInt8::SharedPtr msg) -> void
        { mode = msg->data; });
    sub_phone_upper_depth = this->create_subscription<std_msgs::msg::Float32>(
        "/phone/upper_depth", 10, std::bind(&CoreNode::upper_depth_callback, this, std::placeholders::_1));
    sub_acceleration = this->create_subscription<std_msgs::msg::Bool>(
        "/acceleration", 10, [&](const std_msgs::msg::Bool::SharedPtr msg) -> void
        { acceleration = msg->data; });
    pub_robot_cmd_vel = this->create_publisher<geometry_msgs::msg::Twist>(
        "/robot/cmd_vel", 10);
    pub_robot_under_right_raise = this->create_publisher<std_msgs::msg::Float32>(
        "/robot/under_hand/right_raise", 10);
    pub_robot_under_right_slide = this->create_publisher<std_msgs::msg::Float32>(
        "/robot/under_hand/right_slide", 10);
    pub_robot_under_left_raise = this->create_publisher<std_msgs::msg::Float32>(
        "/robot/under_hand/left_raise", 10);
    pub_robot_under_left_slide = this->create_publisher<std_msgs::msg::Float32>(
        "/robot/under_hand/left_slide", 10);
    if (!udp->initSocket())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize UDP socket");
        return;
    }
    udp->bindSocket(ethernet_config::controller::ip,
                    ethernet_config::controller::port_controller);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(20),
                                     std::bind(&CoreNode::timer_callback, this));
}

void CoreNode::timer_callback()
{
    if (udp->recvPacket(controller_union.code, sizeof(controller_data_union_t)))
    {
    }
}