#include "ararobo_navi/navi_node.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace planner_node
{

    PlannerNode::PlannerNode(const rclcpp::NodeOptions &node_options)
        : Node("planner_node", node_options)
    {
        this->declare_parameter("v_max", 0.5);
        this->declare_parameter("slow_stop", 0.2);
        this->declare_parameter("zero_stop", 0.05);
        this->declare_parameter("position_tolerance", 0.05);
        this->declare_parameter("angle_tolerance", 5.0);
        this->declare_parameter("resolution", 0.1);
        this->declare_parameter("freq", 10.0);

        this->get_parameter("v_max", v_max);
        this->get_parameter("slow_stop", slow_stop_);
        this->get_parameter("zero_stop", zero_stop_);
        this->get_parameter("position_tolerance", position_tolerance_);
        this->get_parameter("angle_tolerance", angle_tolerance_);
        this->get_parameter("resolution", resolution_);
        this->get_parameter("freq", freq);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal", 10, std::bind(&PlannerNode::goal_callback, this, std::placeholders::_1));

        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_out", 10);
        err_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/error_vel", 10);
        path_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/path_marker", 10);

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / freq),
            std::bind(&PlannerNode::timer_callback, this));
        }

    struct Node
    {
        int x, y;
        double g, h;
        Node *parent;

        Node(int x, int y, double g = 0, double h = 0, Node *parent = nullptr)
            : x(x), y(y), g(g), h(h), parent(parent) {}

        double f() const { return g + h; }
        bool operator==(const Node &other) const { return x == other.x && y == other.y; }
    };

    struct CompareNode
    {
        bool operator()(const Node *a, const Node *b) const
        {
            return a->f() > b->f();
        }
    };

    bool isValid(int x, int y, const std::vector<std::vector<int>> &grid)
    {
        return x >= 0 && x < grid.size() && y >= 0 && y < grid[0].size() && grid[x][y] == 0;
    }

    double heuristic(int x1, int y1, int x2, int y2)
    {
        return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
    }

    void PlannerNode::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        goal_rcv = msg;
        RCLCPP_INFO(this->get_logger(), "Received goal: (%.2f, %.2f)", msg->pose.position.x, msg->pose.position.y);
    }

    void PlannerNode::timer_callback()
    {
        if (!goal_rcv)
            return;

        geometry_msgs::msg::TransformStamped transform;
        try
        {
            transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
            return;
        }

        current_pose_.position.x = transform.transform.translation.x;
        current_pose_.position.y = transform.transform.translation.y;
        current_pose_.orientation = transform.transform.rotation;

        double dx = goal_rcv->pose.position.x - current_pose_.position.x;
        double dy = goal_rcv->pose.position.y - current_pose_.position.y;
        double distance = std::sqrt(dx * dx + dy * dy);
        double angle_to_goal = std::atan2(dy, dx);
        double current_yaw = get_Yaw(current_pose_.orientation);
        double angle_error = std::atan2(std::sin(angle_to_goal - current_yaw), std::cos(angle_to_goal - current_yaw));

        geometry_msgs::msg::Twist cmd_vel;
        if (distance > position_tolerance_)
        {
            if (std::fabs(angle_error) > angle_tolerance_ * M_PI / 180.0)
            {
                cmd_vel.angular.z = std::clamp(angle_error, -1.0, 1.0);
                cmd_vel.linear.x = 0.0;
            }
            else
            {
                cmd_vel.angular.z = 0.5 * angle_error;
                cmd_vel.linear.x = std::min(v_max, distance);
            }
        }
        else
        {
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            RCLCPP_INFO(this->get_logger(), "Goal reached.");
        }

        err_pub_->publish(cmd_vel);
    }

    double PlannerNode::get_Yaw(const geometry_msgs::msg::Quaternion &q)
    {
        return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }

} // namespace planner_node
