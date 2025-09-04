

#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <vector>
#include <queue>
#include <memory>
#include <cmath>

namespace aster
{

  struct AStarNode
  {
    int x{0}, y{0};
    double g{0.0}, h{0.0};
    std::shared_ptr<AStarNode> parent{nullptr};
    AStarNode() = default;
    AStarNode(int xx, int yy, double gg, double hh) : x(xx), y(yy), g(gg), h(hh) {}
    double f() const { return g + h; }
  };

  //----------------------------------------------------------------------
  class PlannerNode : public rclcpp::Node
  {
  public:
    PlannerNode();

  private:
    // ---- コールバック ----

    void goal_callback(const geometry_msgs::msg::Pose2D::SharedPtr msg);
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void enable_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void timer_callback();
    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    // ---- A* ----
    bool isValid(int gx, int gy, const std::vector<std::vector<int>> &grid);
    double heuristic(int x1, int y1, int x2, int y2);
    std::vector<std::pair<int, int>> a_star(const std::vector<std::vector<int>> &grid,
                                            int sx, int sy, int gx, int gy);
    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    // ---- 座標変換 ----
    bool worldToGrid(double wx, double wy, int &gx, int &gy) const;
    void gridToWorld(int gx, int gy, double &wx, double &wy) const;

    double get_Yaw(const geometry_msgs::msg::Quaternion &q);
    void draw_path_on_map(const std::vector<std::pair<int, int>> &path);
    void publish_path(const std::vector<std::pair<int, int>> &path); // （cpp側ではインライン実装）

    // ---- ROS I/F ----
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
    nav_msgs::msg::OccupancyGrid latest_map_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr planned_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_with_path_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    // ---- TF ----
    tf2_ros::Buffer::SharedPtr tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    // ---- 状態/設定 ----
    geometry_msgs::msg::Pose2D goal_rcv_;
    geometry_msgs::msg::Pose current_pose_;
    std::vector<std::pair<int, int>> current_path_;
    std::vector<std::vector<int>> grid;

    bool path_ready_{false};
    bool received_map_{false};
    bool nav_enabled_{true};
    bool grid_ready_{false};
    double map_resolution_{1.0};
    double map_origin_x_{0.0};
    double map_origin_y_{0.0};
  };

} // namespace aster

#endif // PLANNER_NODE_HPP_