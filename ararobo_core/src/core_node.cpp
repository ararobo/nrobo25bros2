#include "ararobo_core/core_node.hpp"
#include "ararobo_core/ethernet_config.hpp"

CoreNode::CoreNode() : Node("core_node")
{
    controller_udp = std::make_shared<SimpleUDP>();
    mainboard_udp = std::make_shared<SimpleUDP>();
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
        "/phone/upper_hand/depth", 10, std::bind(&CoreNode::upper_depth_callback, this, std::placeholders::_1));
    sub_phone_acceleration = this->create_subscription<std_msgs::msg::Bool>(
        "/phone/acceleration", 10, [&](const std_msgs::msg::Bool::SharedPtr msg) -> void
        { acceleration = msg->data; });
    sub_phone_auto_mode = this->create_subscription<std_msgs::msg::Bool>(
        "/phone/auto", 10, [&](const std_msgs::msg::Bool::SharedPtr msg) -> void
        { auto_mode = msg->data; });
    pub_robot_cmd_vel = this->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10);
    pub_robot_upper_depth = this->create_publisher<std_msgs::msg::Float32>(
        "/hand/upper/depth", 10);
    pub_robot_upper_width = this->create_publisher<std_msgs::msg::Float32>(
        "/hand/upper/width", 10);
    pub_robot_under_raise = this->create_publisher<std_msgs::msg::Float32>(
        "/hand/under/raise", 10);
    pub_robot_under_slide = this->create_publisher<std_msgs::msg::Float32>(
        "/hand/under/slide", 10);
    pub_robot_lift = this->create_publisher<std_msgs::msg::Float32>(
        "/lift/target", 10);
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
    if (!mainboard_udp->bindSocket(ethernet_config::pc::ip_wifi,
                                   ethernet_config::main_board::port_controller_ble))
    {
        RCLCPP_ERROR(this->get_logger(), "bind error\n");
    }
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
                                     std::bind(&CoreNode::timer_callback, this));
}

void CoreNode::timer_callback()
{
    if (controller_udp->recvPacket(controller_union.code, sizeof(controller_data_union_t)))
    {
        upper_depth = 0.0f;
        upper_width = 0.0f;
        under_hand_raise = 0.0f;
        under_hand_slide = 0.0f;
        cmd_vel_msg.linear.x = controller_union.data.left_stick_x / 127.0f * 1.5f;
        cmd_vel_msg.linear.y = controller_union.data.left_stick_y / 127.0f * 1.5f;
        if (controller_union.data.right_stick_y > 80 || controller_union.data.right_stick_y < -100)
        {
            lift_vel = controller_union.data.right_stick_y / 127.0f * 0.4f;
            cmd_vel_msg.angular.z = 0.0f;
        }
        else
        {
            lift_vel = 0.0f;
            cmd_vel_msg.angular.z = -controller_union.data.right_stick_x / 127.0f * 2.4f;
        }
        if (controller_union.data.buttons.l_up)
        {
            upper_depth += upper_depth_speed;
        }
        if (controller_union.data.buttons.l_down)
        {
            upper_depth -= upper_depth_speed;
        }
        if (controller_union.data.buttons.l_left)
        {
            upper_width -= upper_width_speed;
        }
        if (controller_union.data.buttons.l_right)
        {
            upper_width += upper_width_speed;
        }
        if (controller_union.data.buttons.r_up)
        {
            under_hand_raise += under_hand_raise_speed;
        }
        if (controller_union.data.buttons.r_down)
        {
            under_hand_raise -= under_hand_raise_speed;
        }
        if (controller_union.data.buttons.r_left)
        {
            under_hand_slide -= under_hand_slide_speed;
        }
        if (controller_union.data.buttons.r_right)
        {
            under_hand_slide += under_hand_slide_speed;
        }
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
    lift_pos += lift_vel;
    if (lift_pos > -0.0f)
    {
        lift_pos = -0.0f;
    }
    if (lift_pos < -160.0f)
    {
        lift_pos = -160.0f;
    }

    if (upper_depth_phone < -10.0f)
    {
        upper_depth = upper_depth_phone;
    }

    std_msgs::msg::Float32 lift_msg;
    lift_msg.data = lift_pos;
    pub_robot_lift->publish(lift_msg);
    std_msgs::msg::Float32 upper_depth_msg;
    upper_depth_msg.data = upper_depth;
    pub_robot_upper_depth->publish(upper_depth_msg);
    std_msgs::msg::Float32 upper_width_msg;
    upper_width_msg.data = upper_width;
    pub_robot_upper_width->publish(upper_width_msg);
    std_msgs::msg::Float32 under_hand_raise_msg;
    under_hand_raise_msg.data = under_hand_raise;
    pub_robot_under_raise->publish(under_hand_raise_msg);
    std_msgs::msg::Float32 under_hand_slide_msg;
    under_hand_slide_msg.data = under_hand_slide;
    pub_robot_under_slide->publish(under_hand_slide_msg);
}

void CoreNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
}

void CoreNode::upper_depth_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    upper_depth_phone = msg->data;
}

CoreNode::~CoreNode()
{
    controller_udp->closeSocket();
}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CoreNode>());
    rclcpp::shutdown();
    return 0;
}
