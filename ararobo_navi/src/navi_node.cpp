#include "ararobo_navi/navi_node.hpp"

using std::placeholders::_1;

namespace aster
{
    PlannerNode::PlannerNode()
        : rclcpp::Node("planner_node"), tf_buffer_(std::make_shared<tf2_ros::Buffer>(get_clock())),
          tf_listener_(std::make_unique<tf2_ros::TransformListener>(*tf_buffer_))

    {
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal", 10, std::bind(&PlannerNode::goal_callback, this, _1));
        err_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PlannerNode::timer_callback, this));
    }

    double PlannerNode::get_Yaw(const geometry_msgs::msg::Quaternion &q)
    {
        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        tf2::Matrix3x3 mat(quat);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);
        return yaw;
    }

    bool PlannerNode::isValid(int x, int y, const std::vector<std::vector<int>> &grid)
    {
        return x >= 0 && y >= 0 && x < static_cast<int>(grid[0].size()) &&
               y < static_cast<int>(grid.size()) && grid[y][x] == 0;
    }

    double PlannerNode::heuristic(int x1, int y1, int x2, int y2)
    {
        return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
    }

    std::vector<std::pair<int, int>> PlannerNode::a_star(
        const std::vector<std::vector<int>> &grid, int start_x, int start_y, int goal_x, int goal_y)
    {
        std::priority_queue<AStarNode *, std::vector<AStarNode *>, CompareNode> open_set;
        std::vector<std::vector<bool>> closed_set(grid.size(), std::vector<bool>(grid[0].size(), false));

        AStarNode *start = new AStarNode(start_x, start_y, 0.0, heuristic(start_x, start_y, goal_x, goal_y), nullptr);
        open_set.push(start);

        std::vector<std::pair<int, int>> directions = {
            {1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

        while (!open_set.empty())
        {
            AStarNode *current = open_set.top();
            open_set.pop();

            if (current->x == goal_x && current->y == goal_y)
            {
                std::vector<std::pair<int, int>> path;
                while (current)
                {
                    path.emplace_back(current->x, current->y);
                    current = current->parent;
                }
                std::reverse(path.begin(), path.end());
                return path;
            }

            if (closed_set[current->y][current->x])
            {
                delete current;
                continue;
            }

            closed_set[current->y][current->x] = true;

            for (const auto &dir : directions)
            {
                int nx = current->x + dir.first;
                int ny = current->y + dir.second;

                if (isValid(nx, ny, grid) && !closed_set[ny][nx])
                {
                    double g_new = current->g + ((dir.first == 0 || dir.second == 0) ? 1.0 : std::sqrt(2.0));
                    AStarNode *neighbor = new AStarNode(nx, ny, g_new, heuristic(nx, ny, goal_x, goal_y), current);
                    open_set.push(neighbor);
                }
            }
        }

        return {};
    }

    void PlannerNode::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        goal_rcv = msg;
        path_ready_ = false;
    }

    void PlannerNode::timer_callback()
    {
        if (!goal_rcv)
            return;

        geometry_msgs::msg::TransformStamped tf;
        try
        {
            tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
            return;
        }

        current_pose_.position.x = tf.transform.translation.x;
        current_pose_.position.y = tf.transform.translation.y;
        current_pose_.orientation = tf.transform.rotation;

        int start_x = static_cast<int>(current_pose_.position.x);
        int start_y = static_cast<int>(current_pose_.position.y);
        int goal_x = static_cast<int>(goal_rcv->pose.position.x);
        int goal_y = static_cast<int>(goal_rcv->pose.position.y);

        // 経路計算がまだなら A* を実行！
        if (!path_ready_)
        {
            current_path_ = a_star(grid, start_x, start_y, goal_x, goal_y);
            path_index_ = 0;
            path_ready_ = !current_path_.empty();
            if (!path_ready_)
            {
                RCLCPP_WARN(this->get_logger(), "Failed to find path.");
                return;
            }
        }

        if (path_index_ >= current_path_.size())
        {
            RCLCPP_INFO(this->get_logger(), "Goal reached!");
            return;
        }

        // 現在の目標点
        int target_x = current_path_[path_index_].first;
        int target_y = current_path_[path_index_].second;

        double dx = target_x - current_pose_.position.x;
        double dy = target_y - current_pose_.position.y;
        double dist = std::sqrt(dx * dx + dy * dy);

        if (dist < position_tolerance_)
        {
            path_index_++;
            return;
        }

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = std::min(v_max, dist);
        cmd.angular.z = 2.0 * (std::atan2(dy, dx) - get_Yaw(current_pose_.orientation));
        err_pub_->publish(cmd);
    }

} // namespace aster
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<aster::AStarNode>(); // ←ここは実装クラス名に合わせて！
    rclcpp::shutdown();
    return 0;
}