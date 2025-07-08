#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
class PurePursuitNode : public rclcpp::Node
{
public:
    PurePursuitNode() : Node("pure_pursuit")
    {
        this->declare_parameter("lookahead_distance", 1.0);
        this->get_parameter("lookahead_distance", lookahead_distance);
        RCLCPP_INFO(this->get_logger(), "Lookahead distance: %.2f", lookahead_distance);

        pose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/pose", 10,
            std::bind(&PurePursuitNode::pose_callback, this, std::placeholders::_1));
        path_sub = this->create_subscription<nav_msgs::msg::Path>(
            "/path", 10,
            std::bind(&PurePursuitNode::path_callback, this, std::placeholders::_1));
        cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;

    geometry_msgs::msg::PoseWithCovarianceStamped current_pose;
    nav_msgs::msg::Path path;
    double lookahead_distance;

    // コールバック関数
    // パスと現在の姿勢をメンバ関数に保存
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        path = *msg;
        RCLCPP_INFO(this->get_logger(), "Path received with %zu poses", path.poses.size());
    }

    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "pose_callback called");
        current_pose = *msg;
        RCLCPP_INFO(this->get_logger(), "Current Pose: x=%.2f, y=%.2f",
                    current_pose.pose.pose.position.x, current_pose.pose.pose.position.y);
        compute_control();
    }

    void compute_control()
    {
        geometry_msgs::msg::PoseStamped target;
        bool found = false;

        for (const auto &pose : path.poses)
        {
            double dx = pose.pose.position.x - current_pose.pose.pose.position.x;
            double dy = pose.pose.position.y - current_pose.pose.pose.position.y;
            double dist = std::hypot(dx, dy);
            if (dist >= lookahead_distance)
            {
                target = pose;
                found = true;
                RCLCPP_INFO(this->get_logger(), "Target found: x=%.2f, y=%.2f (dist=%.2f)",
                            target.pose.position.x, target.pose.position.y, dist);
                break;
            }
        }

        if (!found)
        {
            RCLCPP_WARN(this->get_logger(), "No suitable target found.");
            return;
        }

        // 現在の姿勢からyow角を取得
        tf2::Quaternion q(
            current_pose.pose.pose.orientation.x,
            current_pose.pose.pose.orientation.y,
            current_pose.pose.pose.orientation.z,
            current_pose.pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        RCLCPP_INFO(this->get_logger(), "Current Yaw: %.2f rad", yaw);

        // ロボット座標系に変換
        double dx = target.pose.position.x - current_pose.pose.pose.position.x;
        double dy = target.pose.position.y - current_pose.pose.pose.position.y;
        double local_x = std::cos(-yaw) * dx - std::sin(-yaw) * dy;
        double local_y = std::sin(-yaw) * dx + std::cos(-yaw) * dy;
        RCLCPP_INFO(this->get_logger(), "Target in robot frame: x=%.2f, y=%.2f", local_x, local_y);

        geometry_msgs::msg::Twist cmd;
        if (local_x > 1.0)
        {
            local_x = 1.0; // 最大速度制限
        }
        if (local_y > 1.0)
        {
            local_y = 1.0; // 最大速度制限
        }
        cmd.linear.x = local_x; // 前進速度
        cmd.linear.y = local_y; // 側方速度

        RCLCPP_INFO(this->get_logger(), "Publishing cmd_vel: linear.x=%.2f, linear.y=%.2f",
                    cmd.linear.x, cmd.linear.y);

        cmd_pub->publish(cmd);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PurePursuitNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
