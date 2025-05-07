#include "ararobo_can/robot_node.hpp"
#include "ararobo_can/can_config.hpp"

RobotNode::RobotNode()
    : Node("robot_node")
{
    can_id_cmd_vel_ = can_config::encode_id(
        can_config::direction::master,
        can_config::board_type::pc,
        0x00,
        can_config::data_type::pc::cmd_vel);

    declare_parameter("cmd_vel_scale", 3199);
    get_parameter("cmd_vel_scale", cmd_vel_scale_);

    // CANドライバの初期化
    can_driver_ = std::make_shared<CANDriver>();
    can_driver_->init();

    sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        std::bind(&RobotNode::cmd_vel_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "RobotNode initialized");
}

void RobotNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    int16_t cmd_vel_data[3];
    cmd_vel_data[0] = static_cast<int16_t>(msg->linear.x * cmd_vel_scale_);
    cmd_vel_data[1] = static_cast<int16_t>(msg->linear.y * cmd_vel_scale_);
    cmd_vel_data[2] = static_cast<int16_t>(msg->angular.y * cmd_vel_scale_);
    // CANデータの送信
    uint8_t data[6];
    memcpy(data, cmd_vel_data, sizeof(cmd_vel_data));
    can_driver_->send(can_id_cmd_vel_, data, 6);
}

RobotNode::~RobotNode()
{
    // CANドライバの終了処理
    can_driver_->~CANDriver();
    RCLCPP_INFO(this->get_logger(), "RobotNode destroyed");
}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotNode>());
    rclcpp::shutdown();
    return 0;
}
