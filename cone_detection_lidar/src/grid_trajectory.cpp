// ROS
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
// ROS package
#include "cone_detection_lidar/pointcloud_to_grid_core.hpp"
#include "cone_detection_lidar/trajectory.hpp"
#include "cone_detection_lidar/marker.hpp"
#include "cone_detection_lidar/tree.hpp"
// c++
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <vector>
#include <algorithm>

// nav_msgs::msg::OccupancyGrid::Ptr height_grid(new nav_msgs::msg::OccupancyGrid);
auto height_grid = std::make_shared<nav_msgs::msg::OccupancyGrid>();
auto intensity_grid = std::make_shared<nav_msgs::msg::OccupancyGrid>();
class PointCloudToGrid : public rclcpp::Node
{
  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    for (const auto &param : parameters)
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "Param update: " << param.get_name().c_str() << ": " << param.value_to_string().c_str());
      if (param.get_name() == "mapi_topic_name")
      {
        grid_map.mapi_topic_name = param.as_string();
        pub_igrid = this->create_publisher<nav_msgs::msg::OccupancyGrid>(grid_map.mapi_topic_name, 10);
      }
      if (param.get_name() == "maph_topic_name")
      {
        grid_map.maph_topic_name = param.as_string();
        pub_hgrid = this->create_publisher<nav_msgs::msg::OccupancyGrid>(grid_map.maph_topic_name, 10);
      }
      if (param.get_name() == "cloud_in_topic")
      {
        cloud_in_topic = param.as_string();
        sub_pc2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(cloud_in_topic, 10, std::bind(&PointCloudToGrid::lidar_callback, this, std::placeholders::_1));
      }
      if (param.get_name() == "cell_size")
      {
        grid_map.cell_size = param.as_double();
      }
      if (param.get_name() == "position_x")
      {
        grid_map.position_x = param.as_double();
      }
      if (param.get_name() == "position_y")
      {
        grid_map.position_y = param.as_double();
      }
      if (param.get_name() == "length_x")
      {
        grid_map.length_x = param.as_double();
      }
      if (param.get_name() == "length_y")
      {
        grid_map.length_y = param.as_double();
      }
      if (param.get_name() == "intensity_factor")
      {
        grid_map.intensity_factor = param.as_double();
      }
      if (param.get_name() == "height_factor")
      {
        grid_map.height_factor = param.as_double();
      }
      if (param.get_name() == "verbose1")
      {
        verbose1 = param.as_bool();
      }
      if (param.get_name() == "verbose2")
      {
        verbose2 = param.as_bool();
      }
      if (param.get_name() == "search_range_deg")
      {
        search_range_deg = param.as_double();
      }
      if (param.get_name() == "search_resolution_deg")
      {
        search_resolution_deg = param.as_double();
      }
      if (param.get_name() == "search_start_mid_deg")
      {
        search_start_mid_deg = param.as_double();
      }
      if (param.get_name() == "search_length")
      {
        search_length = param.as_double();
      }
      if (param.get_name() == "visualize_grid")
      {
        visualize_grid = param.as_bool();
      }
      if (param.get_name() == "search_iter")
      {
        search_iter = param.as_int();
      }
      // grid_map.frame_out = config.frame_out;
      grid_map.paramRefresh();
    }
    return result;
  }

public:
  PointCloudToGrid() : Node("pointcloud_to_grid_node"), count_(0)
  {
    this->declare_parameter<std::string>("mapi_topic_name", "intensity_grid");
    this->declare_parameter<std::string>("maph_topic_name", "height_grid");
    this->declare_parameter<std::string>("cloud_in_topic", cloud_in_topic);
    this->declare_parameter<float>("cell_size", 0.5);
    this->declare_parameter<float>("position_x", 0.0);
    this->declare_parameter<float>("position_y", 0.0);
    this->declare_parameter<float>("length_x", 20.0);
    this->declare_parameter<float>("length_y", 30.0);
    this->declare_parameter<float>("intensity_factor", 1.0);
    this->declare_parameter<float>("height_factor", 1.0);
    this->declare_parameter<bool>("verbose1", verbose1);
    this->declare_parameter<bool>("verbose2", verbose2);
    this->declare_parameter<double>("search_range_deg", search_range_deg);
    this->declare_parameter<double>("search_resolution_deg", search_resolution_deg);
    this->declare_parameter<double>("search_start_mid_deg", search_start_mid_deg);
    this->declare_parameter<double>("search_length", search_length);
    this->declare_parameter<bool>("visualize_grid", visualize_grid);
    this->declare_parameter<int>("search_iter", search_iter);

    this->get_parameter("mapi_topic_name", grid_map.mapi_topic_name);
    this->get_parameter("maph_topic_name", grid_map.maph_topic_name);
    this->get_parameter("cloud_in_topic", cloud_in_topic);
    this->get_parameter("cell_size", grid_map.cell_size);
    this->get_parameter("position_x", grid_map.position_x);
    this->get_parameter("position_y", grid_map.position_y);
    this->get_parameter("length_x", grid_map.length_x);
    this->get_parameter("length_y", grid_map.length_y);
    this->get_parameter("intensity_factor", grid_map.intensity_factor);
    this->get_parameter("height_factor", grid_map.height_factor);
    this->get_parameter("verbose1", verbose1);
    this->get_parameter("verbose2", verbose2);
    this->get_parameter("search_range_deg", search_range_deg);
    this->get_parameter("search_resolution_deg", search_resolution_deg);
    this->get_parameter("search_start_mid_deg", search_start_mid_deg);
    this->get_parameter("search_length", search_length);
    this->get_parameter("visualize_grid", visualize_grid);
    this->get_parameter("search_iter", search_iter);

    grid_map.paramRefresh();

    pub_igrid = this->create_publisher<nav_msgs::msg::OccupancyGrid>(grid_map.mapi_topic_name, 10);
    pub_hgrid = this->create_publisher<nav_msgs::msg::OccupancyGrid>(grid_map.maph_topic_name, 10);
    pub_marker = this->create_publisher<visualization_msgs::msg::MarkerArray>("debug_marker", 10);
    sub_pc2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(cloud_in_topic, 10, std::bind(&PointCloudToGrid::lidar_callback, this, std::placeholders::_1));
    callback_handle_ = this->add_on_set_parameters_callback(std::bind(&PointCloudToGrid::parametersCallback, this, std::placeholders::_1));
    RCLCPP_INFO_STREAM(this->get_logger(), "pointcloud_to_grid_node has been started.");
    RCLCPP_INFO_STREAM(this->get_logger(), "Subscribing to: " << cloud_in_topic.c_str());
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing to: " << grid_map.mapi_topic_name.c_str() << " and " << grid_map.maph_topic_name.c_str());
  }

private:
  pcl::PointCloud<pcl::PointXYZI>::Ptr crop_pcl(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_in, double min_x_, double min_y_, double max_x_, double max_y_)
  {
    pcl::CropBox<pcl::PointXYZI> crop_fwd;
    crop_fwd.setInputCloud(cloud_in);
    crop_fwd.setMin(Eigen::Vector4f(min_x_, min_y_, -2.0, 1.0));
    crop_fwd.setMax(Eigen::Vector4f(max_x_, max_y_, -0.2, 1.0));
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cropped(new pcl::PointCloud<pcl::PointXYZI>);
    crop_fwd.filter(*cloud_cropped);
    // RCLCPP_INFO_STREAM(this->get_logger(), "crop_fwd: " << cloud_cropped->width * cloud_cropped->height);
    return cloud_cropped;
  }

  void lidar_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*input_msg, *out_cloud);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filt(new pcl::PointCloud<pcl::PointXYZI>);
    cloud_filt = crop_pcl(out_cloud, -80.0, -25.0, +80.0, +25.0); // cloud, min_x, min_y, max_x, max_y
    // Initialize grid
    grid_map.initGrid(intensity_grid);
    grid_map.initGrid(height_grid);
    // width*height/cell_size^2 eg width = 20m, height = 30m, cell_size data size = 6000
    // or cell_num_x * cell_num_ys
    // -128 127 int8[] data
    std::vector<signed char> hpoints(grid_map.cell_num_x * grid_map.cell_num_y);
    std::vector<signed char> ipoints(grid_map.cell_num_x * grid_map.cell_num_y);
    // initialize grid vectors: -128
    for (auto &p : hpoints)
    {
      p = -1;
    }
    for (auto &p : ipoints)
    {
      p = -128;
    }
    for (pcl::PointXYZI p : cloud_filt->points)
    {
      if (p.x > 0.01 || p.x < -0.01)
      {
        if (p.x > grid_map.bottomright_x && p.x < grid_map.topleft_x)
        {
          if (p.y > grid_map.bottomright_y && p.y < grid_map.topleft_y)
          {
            PointXY cell = grid_map.getIndex(p.x, p.y);
            if (cell.x < grid_map.cell_num_x && cell.y < grid_map.cell_num_y)
            {
              ipoints[cell.y * grid_map.cell_num_x + cell.x] = char(p.intensity * grid_map.intensity_factor);
              hpoints[cell.y * grid_map.cell_num_x + cell.x] = +99; // char(p.z * grid_map.height_factor);
            }
            else
            {
              RCLCPP_WARN_STREAM(this->get_logger(), "Cell out of range: " << cell.x << " - " << grid_map.cell_num_x << " ||| " << cell.y << " - " << grid_map.cell_num_y);
            }
          }
        }
      }
    }
    // search_iter is how deep the tree will be (e.g. 6 = 6 levels deep, max 6^2 = 36 nodes in the tree, if every node has 2 children)
    if (search_iter > 6)
    {
      search_iter = 6;
    }
    std::vector<TreeNode *> traj_end_points(1);
    double traj_search_angle = 0.0;
    Tree tree;
    tree.modifyRoot(PointXYori(0.0, 0.0, search_start_mid_deg));
    auto node_actual = tree.getRoot();
    SearchConfig config;
    // iterate through search_iter -> deepness of the tree 
    for (int i = 1; i <= search_iter; i++)
    {
      // traj_end_points are the leaves of the tree
      traj_end_points = tree.getLeaves(node_actual);
      // for every leaf, search for new end points
      for (size_t t = 0; t < traj_end_points.size(); t++)
      {
        config.x_start = traj_end_points[t]->pos.x;
        config.y_start = traj_end_points[t]->pos.y;
        config.search_start_mid_deg = traj_end_points[t]->pos.ori;
        config.search_range_deg = search_range_deg;
        config.search_resolution_deg = search_resolution_deg;
        config.search_length = search_length;
        std::vector<PointXYori> new_end_points = grid_map.angular_search(config, hpoints, traj_search_angle);
        // new end points are added to the tree (typically 1 or 2)
        for (size_t j = 0; j < new_end_points.size(); j++)
        {
          tree.addNode(traj_end_points[t], new_end_points[j]);
        }
      }
    }
    visualization_msgs::msg::MarkerArray mark_array;
    visualization_msgs::msg::Marker debug1_marker, line1_marker;
    init_debug_marker(debug1_marker, traj_end_points[0]->pos.x, traj_end_points[0]->pos.y, 1);
    init_line_marker(line1_marker, 2);
    debug1_marker.header.frame_id = input_msg->header.frame_id;
    debug1_marker.header.stamp = this->now();
    line1_marker.header.frame_id = input_msg->header.frame_id;
    line1_marker.header.stamp = this->now();
    tree.markerTree(tree.root, line1_marker); // add tree to line marker, which can be visualized in rviz
    mark_array.markers.push_back(debug1_marker);
    mark_array.markers.push_back(line1_marker);
    intensity_grid->header.stamp = this->now();
    intensity_grid->header.frame_id = input_msg->header.frame_id;
    intensity_grid->info.map_load_time = this->now();
    intensity_grid->data = ipoints;
    height_grid->header.stamp = this->now();
    height_grid->header.frame_id = input_msg->header.frame_id;
    height_grid->info.map_load_time = this->now();
    height_grid->data = hpoints;
    if (visualize_grid)
    {
      pub_hgrid->publish(*height_grid);
      pub_igrid->publish(*intensity_grid);
    }

    pub_marker->publish(mark_array);
    // pub_hgrid->publish(height_grid);
    if (verbose1)
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "Published " << grid_map.mapi_topic_name.c_str() << " and " << grid_map.maph_topic_name.c_str());
    }
  }

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_igrid, pub_hgrid;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pc2_;
  OnSetParametersCallbackHandle::SharedPtr callback_handle_;
  std::string cloud_in_topic = "nonground";
  bool verbose1 = true, verbose2 = false, visualize_grid = true;
  int search_iter = 4;
  double search_range_deg = 120.0, search_resolution_deg = 10.0, search_start_mid_deg = -180.0;
  double search_length = 10.0;
  // nav_msgs::msg::OccupancyGrid::Ptr intensity_grid = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  // nav_msgs::msg::OccupancyGrid::Ptr intensity_grid(new nav_msgs::msg::OccupancyGrid);
  // nav_msgs::msg::OccupancyGrid::Ptr height_grid = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  GridMapTrajectory grid_map;

  size_t count_;
};
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudToGrid>());
  rclcpp::shutdown();
  return 0;
}
