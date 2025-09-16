#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <cmath>
#include <limits>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

class PurePursuitNode : public rclcpp::Node
{
public:
    PurePursuitNode() : Node("pure_pursuit"), current_vel(0.0)
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        this->declare_parameter("lookahead_distance", 0.5);
        this->declare_parameter("max_vel", 0.5); // m/s
        this->declare_parameter("accel", 0.2);   // m/s^2
        this->get_parameter("lookahead_distance", lookahead_distance);
        this->get_parameter("max_vel", v_max);
        this->get_parameter("accel", accel);

        path_sub = this->create_subscription<nav_msgs::msg::Path>(
            "/path", 10,
            std::bind(&PurePursuitNode::path_callback, this, std::placeholders::_1));
        distance_sub_left = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "distance_left", 10, std::bind(&distance_left, this));
        distance_sub_right = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "distance_right", 10, std::bind(&distance_right, this));

        cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PurePursuitNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Pure Pursuit Node initialized.");
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr distance_sub_left;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr distance_sub_right;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    geometry_msgs::msg::PoseStamped current_pose;
    nav_msgs::msg::Path path;
    double lookahead_distance;
    double v_max, accel;
    double current_vel; // 現在の速度[m/s]
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

        // Find the closest point on the path to the current position
        double min_distance = std::numeric_limits<double>::max();
        int closest_index = 0;

        for (size_t i = 0; i < path.poses.size(); ++i)
        {
            double dx = path.poses[i].pose.position.x - current_pose.pose.position.x;
            double dy = path.poses[i].pose.position.y - current_pose.pose.position.y;
            double dist = std::hypot(dx, dy);

            if (dist < min_distance)
            {
                min_distance = dist;
                closest_index = i;
            }
        }

        RCLCPP_INFO(this->get_logger(), "Closest point index: %d, distance: %.2f", closest_index, min_distance);

        // Look for target point starting from the closest point
        geometry_msgs::msg::PoseStamped target;
        bool found = false;

        for (size_t i = closest_index; i < path.poses.size(); i++)
        {
            double dx = path.poses[i].pose.position.x - current_pose.pose.position.x;
            double dy = path.poses[i].pose.position.y - current_pose.pose.position.y;
            double dist = std::hypot(dx, dy);

            if (dist >= lookahead_distance)
            {
                target = path.poses[i];
                found = true;
                RCLCPP_INFO(this->get_logger(), "Target found: x=%.2f, y=%.2f (dist=%.2f, index=%zu)",
                            target.pose.position.x, target.pose.position.y, dist, i);
                break;
            }
        }

        // If no suitable target found, use the last point in the path
        if (!found)
        {
            target = path.poses.back();
        }
        double dx = target.pose.position.x - current_pose.pose.position.x;
        double dy = target.pose.position.y - current_pose.pose.position.y;
        double dist_to_goal = std::hypot(dx, dy);

        if (dist_to_goal < 0.1) // ゴール到達
        {
            geometry_msgs::msg::Twist stop;
            stop.linear.x = 0.0;
            stop.linear.y = 0.0;
            stop.angular.z = 0.0;
            cmd_pub->publish(stop);
            current_vel = 0.0;
            RCLCPP_INFO(this->get_logger(), "Reached final goal, stopping robot.");
            return;
        }

        // 進行方向ベクトル（正規化）
        double direction_x = dx / dist_to_goal; // 向きを出す
        double direction_y = dy / dist_to_goal;

        // 減速に必要な距離
        double stop_dist = (current_vel * current_vel) / (2 * accel);

        // --- 台形制御 ---
        if (dist_to_goal <= stop_dist)
        {
            // 減速
            current_vel -= accel * 0.1; // Δt = 0.1s
            if (current_vel < 0.0)
                current_vel = 0.0;
        }
        else
        {
            // 加速 or 定速
            if (current_vel < v_max)
            {
                current_vel += accel * 0.1;
                if (current_vel > v_max)
                    current_vel = v_max;
            }
        }

        // --- 出力 ---
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = current_vel * direction_x;
        cmd.linear.y = current_vel * direction_y;
        cmd_pub->publish(cmd);
    }

    void distance_left(std_msgs::msg::Float32MultiArray msg)
    {
        float distance_left0 = msg.data[0];
        float distance_left1 = msg.data[1];
        float distance_left2 = msg.data[2];
        float distance_left3 = msg.data[3];
        const float lefthold = 0.3;
        if (distance_left0 < lefthold)
        {
        }

        if (distance_left1 < lefthold)
        {
        }

        if (distance_left2 < lefthold)
        {
        }

        if (distance_left3 < lefthold)
        {
        }
    }

    void distance_right(const std_msgs::msg::Float32MultiArray msg)
    {
        float distance_right0 = msg.data[0];
        float distance_right1 = msg.data[1];
        float distance_right2 = msg.data[2];
        float distance_right3 = msg.data[3];

        const float righthold = 0.3;

        if (distance_right0 < righthold)
        {
        }

        if (distance_right1 < righthold)
        {
        }

        if (distance_right2 < righthold)
        {
        }

        if (distance_right3 < righthold)
        {
        }
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
