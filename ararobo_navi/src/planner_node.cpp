#include "ararobo_navi/planner_node.hpp"

using std::placeholders::_1;

namespace aster
{
    PlannerNode::PlannerNode()
        : rclcpp::Node("planner_node"), tf_buffer_(std::make_shared<tf2_ros::Buffer>(get_clock())),
          tf_listener_(std::make_unique<tf2_ros::TransformListener>(*tf_buffer_))

    {
        RCLCPP_INFO(this->get_logger(), "Initializing");
        goal_sub_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
            "/nav/goal", 10, std::bind(&PlannerNode::goal_callback, this, _1));
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("path", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PlannerNode::timer_callback, this));
        planned_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("planned_path", 10);
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&PlannerNode::map_callback, this, _1));
        enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/nav/enable", 10, std::bind(&PlannerNode::enable_callback, this, _1));
        // コンストラクタ内でパブリッシャ作成
        map_with_path_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map_with_path", 10);
        RCLCPP_INFO(this->get_logger(), "started");
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
                while (!open_set.empty())
                {
                    delete open_set.top();
                    open_set.pop();
                }
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

    void aster::PlannerNode::goal_callback(const geometry_msgs::msg::Pose2D::SharedPtr msg)
    {
        goal_rcv_ = *msg;
        path_ready_ = false;
        received_map_ = true;
    }
    void PlannerNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        latest_map_ = *msg;   // 最新マップを保存
        received_map_ = true; // 受信フラグを立てる

        int width = msg->info.width;
        int height = msg->info.height;

        // OccupancyGrid.data は一次元配列 → 二次元グリッドに変換
        grid.resize(height);
        for (int y = 0; y < height; ++y)
        {
            grid[y].resize(width);
            for (int x = 0; x < width; ++x)
            {
                int index = y * width + x;
                int value = msg->data[index];

                // 値の分類：0 = free, 100 = obstacle, -1 = unknown
                if (value == 0)
                    grid[y][x] = 0; // 通行可能
                else if (value == 100)
                    grid[y][x] = 1; // 障害物
                else
                    grid[y][x] = 1; // 未知領域も障害物扱い（安全のため）
            }
        }

        RCLCPP_INFO(this->get_logger(), "Map converted to grid (%d x %d)", width, height);
    }

    void PlannerNode::timer_callback()
    {
        if (!path_ready_)
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
        int goal_x = static_cast<int>(goal_rcv_.x); // ← Pose2D型
        int goal_y = static_cast<int>(goal_rcv_.y);

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

            nav_msgs::msg::Path path_msg;
            path_msg.header.stamp = this->now();
            path_msg.header.frame_id = "map";

            for (const auto &pt : current_path_)
            {
                geometry_msgs::msg::PoseStamped pose;
                pose.header = path_msg.header;
                pose.pose.position.x = pt.first;
                pose.pose.position.y = pt.second;
                pose.pose.position.z = 0.0;
                pose.pose.orientation.w = 1.0; // デフォルト
                path_msg.poses.push_back(pose);
            }

            // ✨ここに補間処理を追加！
            nav_msgs::msg::Path interpolated_path;
            interpolated_path.header = path_msg.header;
            double desired_spacing = 0.2;

            interpolated_path.poses.push_back(path_msg.poses.front());
            for (size_t i = 1; i < path_msg.poses.size(); ++i)
            {
                auto p1 = path_msg.poses[i - 1].pose.position;
                auto p2 = path_msg.poses[i].pose.position;
                double dist = std::hypot(p2.x - p1.x, p2.y - p1.y);
                int steps = std::max(1, static_cast<int>(dist / desired_spacing)); // 最低1ステップ
                for (int j = 1; j < steps; ++j)
                {
                    double ratio = static_cast<double>(j) / steps;
                    geometry_msgs::msg::PoseStamped interp;
                    interp.header = path_msg.header;
                    interp.pose.position.x = p1.x + ratio * (p2.x - p1.x);
                    interp.pose.position.y = p1.y + ratio * (p2.y - p1.y);
                    interp.pose.position.z = 0.0;
                    interp.pose.orientation.w = 1.0; // orientation補間は今回は省略
                    interpolated_path.poses.push_back(interp);
                }
                interpolated_path.poses.push_back(path_msg.poses[i]);
            }

            planned_path_pub_->publish(interpolated_path);
            draw_path_on_map(current_path_);
        }
    } // namespace aster
    void PlannerNode::draw_path_on_map(const std::vector<std::pair<int, int>> &path)
    {
        if (!received_map_)
        {
            RCLCPP_WARN(this->get_logger(), "Map not received yet, cannot draw path.");
            return;
        }

        nav_msgs::msg::OccupancyGrid path_map = latest_map_; // 元のマップをコピー（上書きしない）
        int width = path_map.info.width;
        int height = path_map.info.height;

        // 経路に沿ってセルに描き込み
        for (const auto &pt : path)
        {
            int x = pt.first;
            int y = pt.second;

            if (x >= 0 && x < width && y >= 0 && y < height)
            {
                int index = y * width + x;
                path_map.data[index] = 50; // 50など、経路用の独自値（グレー）を使うと分かりやすい！
            }
        }

        map_with_path_pub_->publish(path_map); // マップをパブリッシュ！
        RCLCPP_INFO(this->get_logger(), "Published map with path overlay.");
    }

}
int main(int argc, char **argv)
{
    printf("hey\n");
    rclcpp::init(argc, argv);
    auto node = std::make_shared<aster::PlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}