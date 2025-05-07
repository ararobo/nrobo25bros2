#include "ararobo_can/robot_node.hpp"
#include "ararobo_can/can_config.hpp"

RobotNode::RobotNode()
    : Node("robot_node")
{
    // CANドライバの初期化
    pc_data_slave_ = std::make_shared<PCDataSlave>();
    pc_data_slave_->init();

    sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        std::bind(&RobotNode::cmd_vel_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "RobotNode initialized");
}

void RobotNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    pc_data_slave_->send_cmd_vel(msg->linear.x, msg->linear.y, msg->angular.z);
    RCLCPP_DEBUG(this->get_logger(), "cmd_vel: linear.x: %f, linear.y: %f, angular.z: %f",
                 msg->linear.x, msg->linear.y, msg->angular.z);
}

RobotNode::~RobotNode()
{
    // CANドライバの終了処理
    pc_data_slave_->~PCDataSlave();
    RCLCPP_INFO(this->get_logger(), "RobotNode destroyed");
}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotNode>());
    rclcpp::shutdown();
    return 0;
}
