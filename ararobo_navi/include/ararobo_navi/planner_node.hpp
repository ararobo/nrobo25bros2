#ifndef NAVI_NODE_HPP_
#define NAVI_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <cmath>
#include <queue>
#include <memory>
#include <algorithm>
#include <unordered_set>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/path.hpp> // ←これが必要！
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace aster
{

  class AStarNode
  {
  public:
    int x, y;
    double g, h;
    AStarNode *parent;

    AStarNode(int x_, int y_, double g_, double h_, AStarNode *parent_ = nullptr)
        : x(x_), y(y_), g(g_), h(h_), parent(parent_) {}

    double f() const { return g + h; }

    AStarNode() : x(0), y(0), g(0.0), h(0.0), parent(nullptr) {}

    bool operator==(const AStarNode &other) const
    {
      return x == other.x && y == other.y;
    }
  };

  struct CompareNode
  {
    bool operator()(const AStarNode *a, const AStarNode *b) const
    {
      return a->f() > b->f(); // 小さいf値のノードを優先
    }
  };

  class PlannerNode : public rclcpp::Node
  {
  public:
    PlannerNode();

  private:
    // --- コールバック関数 ---
    void enable_callback(const std_msgs::msg::Bool::SharedPtr msg);       // ナビ有効
    void gate_callback(const std_msgs::msg::Bool::SharedPtr msg);         // ゲート有効
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg); // マップ受信
    void timer_callback();                                                // 周期処理
    void draw_path_on_map(const std::vector<std::pair<int, int>> &path);
    void publish_path(const std::vector<std::pair<int, int>> &path);
    void goal_callback(const geometry_msgs::msg::Pose2D::SharedPtr msg);
    double get_Yaw(const geometry_msgs::msg::Quaternion &q);

    bool isValid(int x, int y, const std::vector<std::vector<int>> &grid);
    double heuristic(int x1, int y1, int x2, int y2);
    double current_x = 0.0;
    double current_y = 0.0;
    std::vector<std::pair<int, int>> a_star(
        const std::vector<std::vector<int>> &grid, int start_x, int start_y, int goal_x, int goal_y);
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr err_pub_;    // /robot/move
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr planned_path_pub_; // /planned_path
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;         // /path
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr tof_right_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr tof_left_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_with_path_pub_;
    nav_msgs::msg::OccupancyGrid latest_map_;
    bool received_map_ = false;

    // --- Subscriber ---
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr goal_sub_;  // /nav/goal
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;       // /nav/enable
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_; // /map
    geometry_msgs::msg::Pose2D goal_rcv_;                                   // ←メンバ変数
    geometry_msgs::msg::Pose current_pose_;
    // --- Timer ---
    rclcpp::TimerBase::SharedPtr timer_; // 周期処理

    // --- TF ---
    tf2_ros::Buffer::SharedPtr tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::vector<std::pair<int, int>> current_path_;
    size_t path_index_ = 0;
    bool path_ready_ = false;

    double position_tolerance_ = 0.2;
    double v_max = 0.5;

    std::vector<std::vector<int>> grid = {
        {0, 0, 0, 1},
        {0, 1, 0, 1},
        {0, 0, 0, 0}};
  };

  void PlannerNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received map with size: %zu", msg->data.size());
  }

  void PlannerNode::enable_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Navigation enabled: %s", msg->data ? "true" : "false");
  }

} // namespace aster

#endif // NAVI_NODE_HPP_
