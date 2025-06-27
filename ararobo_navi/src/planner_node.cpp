
#include "ararobo_navi/planner_node.hpp"
using std::placeholders::_1;

namespace aster
{
    PlannerNode::PlannerNode() : rclcpp::Node("planner_node"),
                                 tf_buffer_(std::make_shared<tf2_ros::Buffer>(get_clock())),
                                 tf_listener_(std::make_unique<tf2_ros::TransformListener>(*tf_buffer_))
    {
        RCLCPP_INFO(get_logger(), "Initializing");
        goal_sub_ = create_subscription<geometry_msgs::msg::Pose2D>("/nav/goal", 10,
                                                                    std::bind(&PlannerNode::goal_callback, this, _1));
        path_pub_ = create_publisher<nav_msgs::msg::Path>("path", 10);
        map_with_path_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("map_with_path", 10);
        map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10,
                                                                     std::bind(&PlannerNode::map_callback, this, _1));
        enable_sub_ = create_subscription<std_msgs::msg::Bool>("/nav/enable", 10,
                                                               std::bind(&PlannerNode::enable_callback, this, _1));
        timer_ = create_wall_timer(std::chrono::milliseconds(100),
                                   std::bind(&PlannerNode::timer_callback, this));
        RCLCPP_INFO(get_logger(), "started");
        pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/pose", 10, std::bind(&PlannerNode::pose_callback, this, _1));
    }

    double PlannerNode::get_Yaw(const geometry_msgs::msg::Quaternion &q)
    {
        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        double r, p, y;
        tf2::Matrix3x3(quat).getRPY(r, p, y);
        return y;
    }

    bool PlannerNode::isValid(int x, int y, const std::vector<std::vector<int>> &g)
    {
        return x >= 0 && y >= 0 && y < (int)g.size() && x < (int)g[0].size() && g[y][x] == 0;
    }

    double PlannerNode::heuristic(int x1, int y1, int x2, int y2)
    {
        return std::hypot(x1 - x2, y1 - y2);
    }
    std::vector<std::pair<int, int>> PlannerNode::a_star(
        const std::vector<std::vector<int>> &g, int sx, int sy, int gx, int gy)
    {
        using NodePtr = std::shared_ptr<AStarNode>;
        struct Cmp
        {
            bool operator()(const NodePtr &a, const NodePtr &b) const { return a->f() > b->f(); }
        };

        std::priority_queue<NodePtr, std::vector<NodePtr>, Cmp> open_set;
        std::vector<std::vector<bool>> closed(g.size(), std::vector<bool>(g[0].size(), false));

        NodePtr start = std::make_shared<AStarNode>(sx, sy, 0.0, heuristic(sx, sy, gx, gy));
        open_set.push(start);

        const std::vector<std::pair<int, int>> dirs{{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

        while (!open_set.empty())
        {
            NodePtr cur = open_set.top();
            open_set.pop();
            if (cur->x == gx && cur->y == gy)
            {
                std::vector<std::pair<int, int>> path;
                for (NodePtr n = cur; n; n = n->parent)
                    path.emplace_back(n->x, n->y);
                std::reverse(path.begin(), path.end());
                return path; // shared_ptr なので自動解放
            }
            if (closed[cur->y][cur->x])
                continue;
            closed[cur->y][cur->x] = true;

            for (auto [dX, dY] : dirs)
            {
                int nx = cur->x + dX, ny = cur->y + dY;
                if (!isValid(nx, ny, g) || closed[ny][nx])
                    continue;
                double g_new = cur->g + ((dX == 0 || dY == 0) ? 1.0 : std::sqrt(2.0));
                NodePtr nb = std::make_shared<AStarNode>(nx, ny, g_new, heuristic(nx, ny, gx, gy));
                nb->parent = cur;
                open_set.push(nb);
            }
        }
        return {};
    }

    void PlannerNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        latest_map_ = *msg;
        received_map_ = true;
        map_resolution_ = msg->info.resolution;
        map_origin_x_ = msg->info.origin.position.x;
        map_origin_y_ = msg->info.origin.position.y;

        int w = msg->info.width, h = msg->info.height;
        grid.assign(h, std::vector<int>(w, 1));
        for (int y = 0; y < h; ++y)
            for (int x = 0; x < w; ++x)
                grid[y][x] = (msg->data[y * w + x] == 0) ? 0 : 1;
        // 再計算トリガ
        path_ready_ = false;
    }

    void PlannerNode::goal_callback(const geometry_msgs::msg::Pose2D::SharedPtr msg)
    {
        goal_rcv_ = *msg;
        path_ready_ = false;
        RCLCPP_INFO(get_logger(), "[GOAL] %.2f %.2f", msg->x, msg->y);
    }
    void PlannerNode::enable_callback(const std_msgs::msg::Bool::SharedPtr msg) { nav_enabled_ = msg->data; }

    bool PlannerNode::worldToGrid(double wx, double wy, int &gx, int &gy) const
    {
        if (!received_map_)
            return false;
        gx = static_cast<int>((wx - map_origin_x_) / map_resolution_);
        gy = static_cast<int>((wy - map_origin_y_) / map_resolution_);
        return gx >= 0 && gy >= 0 && gy < (int)grid.size() && gx < (int)grid[0].size();
    }
    void PlannerNode::gridToWorld(int gx, int gy, double &wx, double &wy) const
    {
        wx = gx * map_resolution_ + map_origin_x_ + map_resolution_ / 2.0;
        wy = gy * map_resolution_ + map_origin_y_ + map_resolution_ / 2.0;
    }

    void PlannerNode::timer_callback()
    {
        if (!received_map_ || !nav_enabled_)
            return;
        int sx, sy, gx, gy;
        if (!worldToGrid(current_pose_.position.x, current_pose_.position.y, sx, sy))
            return;
        if (!worldToGrid(goal_rcv_.x, goal_rcv_.y, gx, gy))
            return;

        // 毎回再計算（もとの guard を削除）
        current_path_ = a_star(grid, sx, sy, gx, gy);
        if (current_path_.empty())
        {
            RCLCPP_WARN(get_logger(), "A* failed");
            return;
        }

        // publish path
        nav_msgs::msg::Path msg;
        msg.header.stamp = now();
        msg.header.frame_id = "map";
        for (auto [cx, cy] : current_path_)
        {
            geometry_msgs::msg::PoseStamped p;
            p.header = msg.header;
            gridToWorld(cx, cy, p.pose.position.x, p.pose.position.y);
            p.pose.orientation.w = 1.0;
            msg.poses.push_back(std::move(p));
        }

        path_pub_->publish(msg);
        draw_path_on_map(current_path_);
    }
    void PlannerNode::draw_path_on_map(const std::vector<std::pair<int, int>> &path)
    {
        if (!received_map_)
            return;
        auto m = latest_map_;
        int w = m.info.width;
        for (auto [gx, gy] : path)
            if (gx >= 0 && gy >= 0 && gy < (int)m.info.height && gx < w)
                m.data[gy * w + gx] = 50;
        map_with_path_pub_->publish(m);
    }
    void PlannerNode::pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        current_pose_.position = msg->pose.pose.position;
        current_pose_.orientation = msg->pose.pose.orientation;

        RCLCPP_INFO(get_logger(), "[POSE] x=%.2f y=%.2f",
                    current_pose_.position.x, current_pose_.position.y);
    }

} // namespace aster
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<aster::PlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}