#include <chrono>
#include <functional>
#include <memory>
#include <string>
// ROS
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
// ROS package
#include "cone_detection_lidar/marker.hpp"

using namespace std::chrono_literals;

class Point
{
public:
    double x, y;
    int neighbor_pts = 0;
    bool core = false;
    int cluster_id = -1;
    /*
        cluster_id is the cluster ID of the point:
        -1: undecided values, not assigned to any cluster
         0: unassigned values: already visited, but not assigned to any cluster
         1: assigned to the first cluster
         2: assigned to the second cluster and so on
    */
};

double distance(const Point &p1, const Point &p2)
{
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

void find_neighbors(std::vector<Point> &points, double eps)
{
    for (Point &p : points)
    {
        for (const Point &q : points)
        {
            if (distance(p, q) <= eps)
            {
                p.neighbor_pts++;
            }
        }
        p.core = p.neighbor_pts >= 2;
    }
}

int find_clusters(std::vector<Point> &points, double eps)
{
    int actual_cluster_id = 0;
    for (int i = 0; i < 10; i++)
    {
        for (Point &p : points)
        {
            if (p.core && p.cluster_id == -1)
            {
                p.cluster_id = actual_cluster_id;
                break;
            }
        }
        for (Point &p : points)
        {
            if (p.cluster_id == actual_cluster_id)
            {
                for (Point &q : points)
                {
                    if (q.core && q.cluster_id == -1 && distance(p, q) <= eps)
                    {
                        q.cluster_id = actual_cluster_id;
                    }
                }
            }
        }
        actual_cluster_id++;
    }
    for (Point &p : points)
    {
        if (!p.core)
        {
            for (const Point &q : points)
            {
                if (q.core && q.cluster_id != -1 && distance(p, q) <= eps)
                {
                    p.cluster_id = q.cluster_id;
                }
            }
        }
    }
    return actual_cluster_id;
}

class DetectionSimple : public rclcpp::Node
{
    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";
        for (const auto &param : parameters)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "Param update: " << param.get_name().c_str() << ": " << param.value_to_string().c_str());
            if (param.get_name() == "cloud_in_topic")
            {
                cloud_in_topic = param.as_string();
                // sub_pc2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(cloud_in_topic, 10, std::bind(&PointCloudToGrid::lidar_callback, this, std::placeholders::_1));
            }
            if (param.get_name() == "points_out_topic")
            {
                points_out_topic = param.as_string();
                pub_lidar_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(points_out_topic, 10);
            }
            if (param.get_name() == "marker_out_topic")
            {
                marker_out_topic = param.as_string();
                pub_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_out_topic, 10);
            }
            if (param.get_name() == "verbose1")
            {
                verbose1 = param.as_bool();
            }
            if (param.get_name() == "verbose2")
            {
                verbose2 = param.as_bool();
            }
            if (param.get_name() == "minX")
            {
                minX = param.as_double();
            }
            if (param.get_name() == "minY")
            {
                minY = param.as_double();
            }
            if (param.get_name() == "minZ")
            {
                minZ = param.as_double();
            }
            if (param.get_name() == "maxX")
            {
                maxX = param.as_double();
            }
            if (param.get_name() == "maxY")
            {
                maxY = param.as_double();
            }
            if (param.get_name() == "maxZ")
            {
                maxZ = param.as_double();
            }
            if (param.get_name() == "cluster_points_max")
            {
                cluster_points_max = param.as_int();
            }
            if (param.get_name() == "cluster_points_min")
            {
                cluster_points_min = param.as_int();
            }
        }
        return result;
    }

public:
    DetectionSimple() : Node("detection_simple"), count_(0)
    {
        this->declare_parameter<std::string>("cloud_in_topic", cloud_in_topic);
        this->declare_parameter<std::string>("points_out_topic", "clustered_points");
        this->declare_parameter<std::string>("marker_out_topic", "clustered_marker");
        this->declare_parameter<bool>("verbose1", verbose1);
        this->declare_parameter<bool>("verbose2", verbose2);
        this->declare_parameter<float>("minX", minX);
        this->declare_parameter<float>("minY", minY);
        this->declare_parameter<float>("minZ", minZ);
        this->declare_parameter<float>("maxX", maxX);
        this->declare_parameter<float>("maxY", maxY);
        this->declare_parameter<float>("maxZ", maxZ);
        this->declare_parameter<int>("cluster_points_max", cluster_points_max);
        this->declare_parameter<int>("cluster_points_min", cluster_points_min);
        this->declare_parameter<double>("eps", eps);

        this->get_parameter("cloud_in_topic", cloud_in_topic);
        this->get_parameter("points_out_topic", points_out_topic);
        this->get_parameter("marker_out_topic", marker_out_topic);
        this->get_parameter("verbose1", verbose1);
        this->get_parameter("verbose2", verbose2);
        this->get_parameter("minX", minX);
        this->get_parameter("minY", minY);
        this->get_parameter("minZ", minZ);
        this->get_parameter("maxX", maxX);
        this->get_parameter("maxY", maxY);
        this->get_parameter("maxZ", maxZ);
        this->get_parameter("cluster_points_max", cluster_points_max);
        this->get_parameter("cluster_points_min", cluster_points_min);
        this->get_parameter("eps", eps);

        pub_lidar_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(points_out_topic, 10);
        pub_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_out_topic, 10);
        sub_lidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(cloud_in_topic, 10, std::bind(&DetectionSimple::lidar_callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(500ms, std::bind(&DetectionSimple::timer_callback, this));
        callback_handle_ = this->add_on_set_parameters_callback(std::bind(&DetectionSimple::parametersCallback, this, std::placeholders::_1));
    }

private:
    void timer_callback()
    {
    }

    void lidar_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg)
    {
        visualization_msgs::msg::MarkerArray mark_array;
        // Convert to PCL data type
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*input_msg, *cloud);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
        int original_size = cloud->width * cloud->height;
        // Filter out points outside of the box
        pcl::CropBox<pcl::PointXYZI> crop;
        crop.setInputCloud(cloud);
        crop.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
        crop.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
        crop.filter(*cloud);

        if (verbose1)
        {
            // print the length of the pointcloud
            RCLCPP_INFO_STREAM(this->get_logger(), "PointCloud in: " << original_size << " reduced size before cluster: " << cloud->width * cloud->height);
        }

        // DBSCAN
        // find neighbors in cloud
        std::vector<Point> points;

        for (const pcl::PointXYZI &p : cloud->points)
        {
            {
                Point point;
                point.x = p.x;
                point.y = p.y;
                points.push_back(point);
            }
        }

        // mark_array.action = visualization_msgs::msg::MarkerArray::DELETEALL;

        find_neighbors(points, eps);
        // find clusters in cloud
        int num_of_clusters = find_clusters(points, eps);
        // create a vector of points for each cluster
        std::vector<double> center_x(num_of_clusters), center_y(num_of_clusters);
        std::vector<int> count(num_of_clusters);
        // init
        for (int i = 0; i < num_of_clusters; i++)
        {
            center_x[i] = 0.0;
            center_y[i] = 0.0;
            count[i] = 0;
        }
        for (const Point &p : points)
        {
            if (p.cluster_id > 0)
            {
                count[p.cluster_id]++;
            }
        }
        {
            int min_count = 1000000;
            int max_count = 0;
            for (int i = 0; i < num_of_clusters; i++)
            {
                if (count[i] > max_count)
                {
                    max_count = count[i];
                }
                if (count[i] < min_count)
                {
                    min_count = count[i];
                }
            }
            if (verbose2)
            {
                RCLCPP_INFO_STREAM(this->get_logger(), "min_count: " << min_count << " max_count: " << max_count);
            }
        }

        // convert to PointXYZI
        for (const Point &p : points)
        {
            if (p.cluster_id > 0)
            {
                if (count[p.cluster_id] < cluster_points_max and count[p.cluster_id] > cluster_points_min)
                {
                    pcl::PointXYZI point;
                    point.x = p.x;
                    point.y = p.y;
                    point.z = 0.0;
                    point.intensity = p.cluster_id;
                    center_x[p.cluster_id] += p.x;
                    center_y[p.cluster_id] += p.y;
                    cloud_filtered->points.push_back(point);
                }
            }
        }
        for (int i = 0; i < num_of_clusters; i++)
        {
            if (count[i] < cluster_points_max and count[i] > cluster_points_min)
            {
                center_x[i] /= count[i];
                center_y[i] /= count[i];
                visualization_msgs::msg::Marker center_marker;
                init_center_marker(center_marker, center_x[i], center_y[i], i);
                center_marker.header.frame_id = input_msg->header.frame_id;
                center_marker.header.stamp = this->now();
                mark_array.markers.push_back(center_marker);
            }
        }

        // Convert to ROS data type
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*cloud_filtered, output_msg);
        // Add the same frame_id as the input, it is not included in pcl PointXYZI
        output_msg.header.frame_id = input_msg->header.frame_id;
        // Publish the data as a ROS message
        pub_lidar_->publish(output_msg);
        pub_marker_->publish(mark_array);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_lidar_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;
    std::string cloud_in_topic = "nonground", points_out_topic, marker_out_topic;
    float minX = -80.0, minY = -25.0, minZ = -2.0;
    float maxX = +80.0, maxY = +25.0, maxZ = -0.15;
    double eps = 0.3;
    bool verbose1 = false, verbose2 = false;
    int cluster_points_min = 2, cluster_points_max = 50;
    size_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectionSimple>());
    rclcpp::shutdown();
    return 0;
}