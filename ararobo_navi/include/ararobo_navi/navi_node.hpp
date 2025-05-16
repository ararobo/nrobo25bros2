#ifndef NAVI_NODE_HPP_
#define NAVI_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>

namespace planner_node
{
  class PlannerNode : public rclcpp::Node
  {
  public:
    explicit PlannerNode(const rclcpp::NodeOptions &node_options);

  private:
    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void timer_callback();

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr err_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    geometry_msgs::msg::PoseStamped::SharedPtr goal_rcv;
    geometry_msgs::msg::Pose current_pose_;

    double slow_stop_;
    double zero_stop_;
    double v_max;
    double freq;
    double position_tolerance_ = 0.05;
    double angle_tolerance_ = 5.0;
    double resolution_;
    double geat_pud;

    double get_Yaw(const geometry_msgs::msg::Quaternion &q)
    {
      return atan2(2.0 * (q.w * q.z + q.x * q.y),
                   1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }
  };
}

#endif // NAVI_NODE_HPP_
