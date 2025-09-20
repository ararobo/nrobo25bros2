#include "ararobo_core/core_node.hpp"
#include "ararobo_core/ethernet_config.hpp"

CoreNode::CoreNode() : Node("core_node")
{
    udp = std::make_shared<SimpleUDP>();
    trapezoidal_controller_x = std::make_shared<TrapezoidalController<float>>();
    trapezoidal_controller_y = std::make_shared<TrapezoidalController<float>>();
    trapezoidal_controller_z = std::make_shared<TrapezoidalController<float>>();
    trapezoidal_controller_x->set_control_cycle(20);
    trapezoidal_controller_y->set_control_cycle(20);
    trapezoidal_controller_z->set_control_cycle(20);
    cmd_vel_msg.linear.x = 0.0;
    cmd_vel_msg.linear.y = 0.0;
    cmd_vel_msg.linear.z = 0.0;
    cmd_vel_msg.angular.x = 0.0;
    cmd_vel_msg.angular.y = 0.0;
    cmd_vel_msg.angular.z = 0.0;
    sub_phone_cmd_vel = this->create_subscription<geometry_msgs::msg::Twist>(
        "/phone/cmd_vel", 10, std::bind(&CoreNode::cmd_vel_callback, this, std::placeholders::_1));
    sub_phone_mode = this->create_subscription<std_msgs::msg::UInt8>(
        "/phone/mode", 10, [&](const std_msgs::msg::UInt8::SharedPtr msg) -> void
        { mode = msg->data; });
    sub_phone_upper_depth = this->create_subscription<std_msgs::msg::Float32>(
        "/phone/upper_depth", 10, std::bind(&CoreNode::upper_depth_callback, this, std::placeholders::_1));
    sub_phone_acceleration = this->create_subscription<std_msgs::msg::Bool>(
        "/phone/acceleration", 10, [&](const std_msgs::msg::Bool::SharedPtr msg) -> void
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
        cmd_vel_msg.linear.x = controller_union.data.left_stick_x / 127.0f * 1.5f;
        cmd_vel_msg.linear.y = controller_union.data.left_stick_y / 127.0f * 1.5f;
        cmd_vel_msg.angular.z = controller_union.data.right_stick_x / 127.0f * 1.5f;
    }
    float x = cmd_vel_msg.linear.x;
    float y = cmd_vel_msg.linear.y;
    float z = cmd_vel_msg.angular.z;
    if (acceleration)
    {
        x = trapezoidal_controller_x->trapezoidal_control(x, 0.001f);
        y = trapezoidal_controller_y->trapezoidal_control(y, 0.001f);
        z = trapezoidal_controller_z->trapezoidal_control(z, 0.001f);
    }
    cmd_vel_msg.linear.x = x;
    cmd_vel_msg.linear.y = y;
    cmd_vel_msg.angular.z = z;
    pub_robot_cmd_vel->publish(cmd_vel_msg);
    RCLCPP_INFO(this->get_logger(), "cmd_vel x: %.2f y: %.2f z: %.2f", x, y, z);
}

void CoreNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
}

void CoreNode::upper_depth_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CoreNode>());
    rclcpp::shutdown();
    return 0;
}
