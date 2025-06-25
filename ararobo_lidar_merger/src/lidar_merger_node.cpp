#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h> // 必要に応じて

class LidarMergerNode : public rclcpp::Node
{
public:
    LidarMergerNode()
        : Node("lidar_merger_node"), // ノード名をlidar_merger_nodeに変更
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_),
          laser_projector_()
    {
        // パラメータの宣言と取得
        this->declare_parameter("urg_scan_topic", "/urg_scan");
        this->declare_parameter("ydlidar_scan_topic", "/ydlidar_scan");
        this->declare_parameter("output_scan_topic", "/scan");
        this->declare_parameter("target_frame", "base_link");
        this->declare_parameter("min_angle", -M_PI);
        this->declare_parameter("max_angle", M_PI);
        this->declare_parameter("angle_increment", M_PI / 180.0); // 1度刻み
        this->declare_parameter("min_range", 0.05);
        this->declare_parameter("max_range", 100.0);

        std::string urg_scan_topic = this->get_parameter("urg_scan_topic").as_string();
        std::string ydlidar_scan_topic = this->get_parameter("ydlidar_scan_topic").as_string();
        output_scan_topic_ = this->get_parameter("output_scan_topic").as_string();
        target_frame_ = this->get_parameter("target_frame").as_string();
        min_angle_ = this->get_parameter("min_angle").as_double();
        max_angle_ = this->get_parameter("max_angle").as_double();
        angle_increment_ = this->get_parameter("angle_increment").as_double();
        min_range_ = this->get_parameter("min_range").as_double();
        max_range_ = this->get_parameter("max_range").as_double();

        RCLCPP_INFO(this->get_logger(), "Subscribing to URG scan topic: %s", urg_scan_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Subscribing to YDLIDAR scan topic: %s", ydlidar_scan_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing to output scan topic: %s", output_scan_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Target frame for TF transformations: %s", target_frame_.c_str());

        // LaserScanサブスクライバの設定
        urg_sub_.subscribe(this, urg_scan_topic);
        ydlidar_sub_.subscribe(this, ydlidar_scan_topic);

        // ApproximateTimeSynchronizerの設定
        // キューサイズを調整する必要があるかもしれません。
        // デバイスのレートやネットワークの安定性に応じて増やすことを検討してください。
        synchronizer_ = std::make_shared<message_filters::Synchronizer<ApproximateTime>>(ApproximateTime(10), urg_sub_, ydlidar_sub_);
        synchronizer_->registerCallback(std::bind(&LidarMergerNode::sync_callback, this, std::placeholders::_1, std::placeholders::_2));

        // 統合されたLaserScanのパブリッシャ
        scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(output_scan_topic_, 10);
    }

private:
    void sync_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& urg_scan_msg,
                       const sensor_msgs::msg::LaserScan::ConstSharedPtr& ydlidar_scan_msg)
    {
        RCLCPP_DEBUG(this->get_logger(), "Received synchronized URG and YDLIDAR scans.");

        try {
            // 1. LaserScanをPointCloud2に変換 (URG)
            sensor_msgs::msg::PointCloud2 urg_cloud_msg;
            laser_projector_.projectLaser(*urg_scan_msg, urg_cloud_msg);
            RCLCPP_DEBUG(this->get_logger(), "Converted URG LaserScan to PointCloud2.");

            // 2. LaserScanをPointCloud2に変換 (YDLIDAR)
            sensor_msgs::msg::PointCloud2 ydlidar_cloud_msg;
            laser_projector_.projectLaser(*ydlidar_scan_msg, ydlidar_cloud_msg);
            RCLCPP_DEBUG(this->get_logger(), "Converted YDLIDAR LaserScan to PointCloud2.");

            // 3. TF2による座標変換 (URG)
            sensor_msgs::msg::PointCloud2 transformed_urg_cloud;
            tf_buffer_.transform(urg_cloud_msg, transformed_urg_cloud, target_frame_, tf2::Duration(RCL_MS_TO_NS(100))); // 100ms timeout
            RCLCPP_DEBUG(this->get_logger(), "Transformed URG PointCloud2 to %s frame.", target_frame_.c_str());

            // 4. TF2による座標変換 (YDLIDAR)
            sensor_msgs::msg::PointCloud2 transformed_ydlidar_cloud;
            tf_buffer_.transform(ydlidar_cloud_msg, transformed_ydlidar_cloud, target_frame_, tf2::Duration(RCL_MS_TO_NS(100))); // 100ms timeout
            RCLCPP_DEBUG(this->get_logger(), "Transformed YDLIDAR PointCloud2 to %s frame.", target_frame_.c_str());

            // 5. PointCloud2の結合
            pcl::PointCloud<pcl::PointXYZ> combined_pcl_cloud;
            pcl::PointCloud<pcl::PointXYZ> urg_pcl_cloud;
            pcl::PointCloud<pcl::PointXYZ> ydlidar_pcl_cloud;

            pcl::fromROSMsg(transformed_urg_cloud, urg_pcl_cloud);
            pcl::fromROSMsg(transformed_ydlidar_cloud, ydlidar_pcl_cloud);

            combined_pcl_cloud = urg_pcl_cloud;
            combined_pcl_cloud += ydlidar_pcl_cloud; // PCL PointCloudの結合

            RCLCPP_DEBUG(this->get_logger(), "Combined PointClouds. Total points: %zu", combined_pcl_cloud.points.size());

            // 6. PointCloud2からLaserScanへの再変換
            sensor_msgs::msg::LaserScan integrated_scan;
            integrated_scan.header.stamp = urg_scan_msg->header.stamp; // 同期されたタイムスタンプを使用
            integrated_scan.header.frame_id = target_frame_;
            integrated_scan.angle_min = min_angle_;
            integrated_scan.angle_max = max_angle_;
            integrated_scan.angle_increment = angle_increment_;
            integrated_scan.time_increment = 0.0; // PCLから再構築するため、基本的には0
            integrated_scan.scan_time = urg_scan_msg->scan_time; // URGのscan_timeを流用
            integrated_scan.range_min = min_range_;
            integrated_scan.range_max = max_range_;

            // rangesとintensitiesの初期化
            int num_beams = static_cast<int>((integrated_scan.angle_max - integrated_scan.angle_min) / integrated_scan.angle_increment) + 1;
            integrated_scan.ranges.assign(num_beams, std::numeric_limits<float>::infinity());
            integrated_scan.intensities.assign(num_beams, 0.0); // 輝度情報がない場合は0で初期化

            for (const auto& point : combined_pcl_cloud.points)
            {
                // 無効な点を除外
                if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)) {
                    continue;
                }

                double angle = atan2(point.y, point.x);
                double range = sqrt(point.x * point.x + point.y * point.y + point.z * point.z); // 3D距離

                // 範囲外の点を無視
                if (range < integrated_scan.range_min || range > integrated_scan.range_max) {
                    continue;
                }

                // 角度をビームインデックスに変換
                if (angle >= integrated_scan.angle_min && angle <= integrated_scan.angle_max)
                {
                    int index = static_cast<int>((angle - integrated_scan.angle_min) / integrated_scan.angle_increment);

                    // 同じ角度に複数の点がある場合、最も近い距離を採用
                    if (range < integrated_scan.ranges[index])
                    {
                        integrated_scan.ranges[index] = static_cast<float>(range);
                    }
                }
            }
            RCLCPP_DEBUG(this->get_logger(), "Reconstructed LaserScan from combined PointCloud.");

            // 7. 統合されたLaserScanの配信
            scan_publisher_->publish(integrated_scan);
            RCLCPP_DEBUG(this->get_logger(), "Published integrated LaserScan.");

        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF Transform error: %s", ex.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "An error occurred: %s", e.what());
        }
    }

    // Message Filters
    message_filters::Subscriber<sensor_msgs::msg::LaserScan> urg_sub_;
    message_filters::Subscriber<sensor_msgs::msg::LaserScan> ydlidar_sub_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan> ApproximateTime;
    std::shared_ptr<message_filters::Synchronizer<ApproximateTime>> synchronizer_;

    // TF2
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Laser Geometry
    laser_geometry::LaserProjection laser_projector_;

    // ROS Publishers
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;

    // Parameters
    std::string output_scan_topic_;
    std::string target_frame_;
    double min_angle_;
    double max_angle_;
    double angle_increment_;
    double min_range_;
    double max_range_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarMergerNode>());
    rclcpp::shutdown();
    return 0;
}