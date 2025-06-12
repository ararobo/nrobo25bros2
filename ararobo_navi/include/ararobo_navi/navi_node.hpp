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
    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void timer_callback();
    double get_Yaw(const geometry_msgs::msg::Quaternion &q);

    bool isValid(int x, int y, const std::vector<std::vector<int>> &grid);
    double heuristic(int x1, int y1, int x2, int y2);
    std::vector<std::pair<int, int>> a_star(
        const std::vector<std::vector<int>> &grid, int start_x, int start_y, int goal_x, int goal_y);

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr err_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Pose current_pose_;
    geometry_msgs::msg::PoseStamped::SharedPtr goal_rcv;

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

} // namespace aster

#endif // NAVI_NODE_HPP_
