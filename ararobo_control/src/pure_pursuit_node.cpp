#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

class PurePursuitNode : public rclcpp::Node
{
public:
    PurePursuitNode() : Node("pure_pursuit")
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        this->declare_parameter("lookahead_distance", 1.0);
        this->get_parameter("lookahead_distance", lookahead_distance);
        RCLCPP_INFO(this->get_logger(), "Lookahead distance: %.2f", lookahead_distance);

        path_sub = this->create_subscription<nav_msgs::msg::Path>(
            "/path", 10,
            std::bind(&PurePursuitNode::path_callback, this, std::placeholders::_1));
        cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // タイマーのコールバックを timer_callback() に変更
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PurePursuitNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Pure Pursuit Node initialized.");
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    geometry_msgs::msg::PoseStamped current_pose;
    nav_msgs::msg::Path path;
    double lookahead_distance;

    void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        path = *msg;
        RCLCPP_INFO(this->get_logger(), "Path received with %zu poses", path.poses.size());
    }

    // compute_control -> timer_callback に変更
    void timer_callback()
    {
        geometry_msgs::msg::PoseStamped pose;
        try
        {
            // Get current pose
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
            pose.pose.position.x = transform.transform.translation.x;
            pose.pose.position.y = transform.transform.translation.y;
            pose.pose.position.z = transform.transform.translation.z;
            pose.pose.orientation.x = transform.transform.rotation.x;
            pose.pose.orientation.y = transform.transform.rotation.y;
            pose.pose.orientation.z = transform.transform.rotation.z;
            pose.pose.orientation.w = transform.transform.rotation.w;
            current_pose = pose;
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not get transform: %s", ex.what());
            return;
        }

        if (path.poses.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No path received yet. Skipping control.");
            return;
        }

        geometry_msgs::msg::PoseStamped target;
        bool found = false;

        for (const auto &path_pose : path.poses)
        {
            double dx = path_pose.pose.position.x - current_pose.pose.position.x;
            double dy = path_pose.pose.position.y - current_pose.pose.position.y;
            double dist = std::hypot(dx, dy);
            if (dist >= lookahead_distance)
            {
                target = path_pose;
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

        tf2::Quaternion q(
            current_pose.pose.orientation.x,
            current_pose.pose.orientation.y,
            current_pose.pose.orientation.z,
            current_pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        RCLCPP_INFO(this->get_logger(), "Current Yaw: %.2f rad", yaw);

        double dx = target.pose.position.x - current_pose.pose.position.x;
        double dy = target.pose.position.y - current_pose.pose.position.y;
        double local_x = std::cos(-yaw) * dx - std::sin(-yaw) * dy;
        double local_y = std::sin(-yaw) * dx + std::cos(-yaw) * dy;
        RCLCPP_INFO(this->get_logger(), "Target in robot frame: x=%.2f, y=%.2f", local_x, local_y);

        geometry_msgs::msg::Twist cmd;
        if (local_x > 1.0)
            local_x = 1.0;
        if (local_y > 1.0)
            local_y = 1.0;
        cmd.linear.x = local_x;
        cmd.linear.y = local_y;

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
