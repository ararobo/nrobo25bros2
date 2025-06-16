#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <nav_msgs/msg/path.hpp>
#include <cmath>

class PurePursuitNode : public rclcpp::Node
{
public:
    PurePursuitNode() : Node("pure_pursuit")
    {
        this->declare_parameter("lookahead_distance", 1.0);
        this->get_parameter("lookahead_distance", lookaheaddistance);

        posesub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/current_pose", 10,
            std::bind(&PurePursuitNode::poseCallback, this, std::placeholders::_1));
        pathsub = this->create_subscription<nav_msgs::msg::Path>(
            "/planned_path", 10,
            std::bind(&PurePursuitNode::pathCallback, this, std::placeholders::_1));
        cmdpub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr posesub;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr pathsub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdpub;

    geometry_msgs::msg::PoseStamped currentpose;
    nav_msgs::msg::Path path;
    double lookaheaddistance;

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        path = *msg;
    }

    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        currentpose = *msg;
        computeControl();
    }
    void computeControl()
    {
        geometry_msgs::msg::PoseStamped target;
        bool found = false;

        for (const auto &pose : path.poses)
        {
            double dx = pose.pose.position.x - currentpose.pose.position.x;
            double dy = pose.pose.position.y - currentpose.pose.position.y;
            if (std::hypot(dx, dy) >= lookaheaddistance)
            {
                target = pose;
                found = true;
                break;
            }
        }

        if (!found)
            return;

        // 現在の姿勢からyow角を取得
        tf2::Quaternion q(
            currentpose.pose.orientation.x,
            currentpose.pose.orientation.y,
            currentpose.pose.orientation.z,
            currentpose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // 目標との角度差を計算
        double dx = target.pose.position.x - currentpose.pose.position.x;
        double dy = target.pose.position.y - currentpose.pose.position.y;
        double angle_to_target = std::atan2(dy, dx);
        double alpha = angle_to_target - yaw;

        double steering = std::atan2(2.0 * std::sin(alpha), lookaheaddistance);

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 1.0; // 定速
        cmd.angular.z = steering;
        cmdpub->publish(cmd);
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