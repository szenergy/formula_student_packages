#include <iostream>
#include <vector>
#include <random>
#include <cmath>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "pcl/filters/crop_box.h"
#include <math.h> // to have M_PI

// Helper function to generate a random double within a given range
double dRand(double min, double max, std::default_random_engine& generator) {
    std::uniform_real_distribution<double> distribution(min, max);
    return distribution(generator);
}

// Function to generate distorted circle points
std::vector<std::pair<double, double>> generateDistortedCirclePoints(int num_keypoints, double radius, double distortion_factor, unsigned int seed) {
    // Initialize a vector to store the points
    std::vector<std::pair<double, double>> points;
    
    // Set up the random number generator with the seed
    std::default_random_engine generator(seed);

    // Calculate the step size for the angle based on the number of keypoints
    double angle_step = 2 * M_PI / num_keypoints;

    // Loop over the number of keypoints
    for (int i = 0; i < num_keypoints; ++i) {
        // Calculate the current angle
        double angle = i * angle_step;
        
        // Calculate the x and y coordinates for the current point on the undistorted circle
        double x = radius * cos(angle);
        double y = radius * sin(angle);

        // Calculate a random offset for the angle and distance based on the distortion factor
        double angle_offset = dRand(-distortion_factor, distortion_factor, generator) * M_PI;
        double distance_offset = dRand(-distortion_factor, distortion_factor, generator) * radius;

        // Apply the distortion to the x and y coordinates
        x += distance_offset * cos(angle + angle_offset);
        y += distance_offset * sin(angle + angle_offset);

        // Add the distorted point to the vector
        points.emplace_back(x, y);
    }

    // Close the loop by appending the first point again
    points.push_back(points.front());
    
    // Return the vector of distorted circle points
    return points;
}

// Function to interpolate points using a Catmull-Rom spline
std::vector<std::pair<double, double>> interpolateSpline(const std::vector<std::pair<double, double>>& points, int num_interpolated_points) {
    // Initialize a vector to store the interpolated points
    std::vector<std::pair<double, double>> interpolated_points;

    // Define the Catmull-Rom spline function
    auto catmullRom = [](const std::pair<double, double>& p0, const std::pair<double, double>& p1, const std::pair<double, double>& p2, const std::pair<double, double>& p3, double t) {
        // Calculate the t-squared and t-cubed terms
        double t2 = t * t;
        double t3 = t2 * t;

        // Calculate the x and y coordinates for the interpolated point
        double x = 0.5 * ((2 * p1.first) + (-p0.first + p2.first) * t + (2 * p0.first - 5 * p1.first + 4 * p2.first - p3.first) * t2 + (-p0.first + 3 * p1.first - 3 * p2.first + p3.first) * t3);
        double y = 0.5 * ((2 * p1.second) + (-p0.second + p2.second) * t + (2 * p0.second - 5 * p1.second + 4 * p2.second - p3.second) * t2 + (-p0.second + 3 * p1.second - 3 * p2.second + p3.second) * t3);

        // Return the interpolated point
        return std::make_pair(x, y);
    };

    int n = points.size() - 1; // -1 because the last point is a duplicate of the first one

    // Loop over the input points, treating the points as a closed loop
    for (int i = 0; i < n; ++i) {
        // For each input point, generate the specified number of interpolated points
        for (int j = 0; j < num_interpolated_points; ++j) {
            // Calculate the parameter t for the current interpolated point
            double t = static_cast<double>(j) / num_interpolated_points;
            // Add the interpolated point to the vector
            interpolated_points.push_back(catmullRom(
                points[(i - 1 + n) % n], 
                points[i % n], 
                points[(i + 1) % n], 
                points[(i + 2) % n], 
                t
            ));
        }
    }

    // Close the loop by appending the first interpolated point again
    interpolated_points.push_back(interpolated_points.front());

    // Return the vector of interpolated points
    return interpolated_points;
}

// Function to generate cone points around the racetrack
std::vector<std::tuple<double, double, int>> generateConePoints(const std::vector<std::pair<double, double>>& track_points, double cone_spacing, double cone_distance, int num_noise_points, double max_noise_distance) {
  std::vector<std::tuple<double, double, int>> cone_points;
  double accumulated_distance = 0.0;

  for (size_t i = 0; i < track_points.size() - 1; ++i) {
      double x1 = track_points[i].first;
      double y1 = track_points[i].second;
      double x2 = track_points[i + 1].first;
      double y2 = track_points[i + 1].second;
  
      // Direction vector
      double dx = x2 - x1;
      double dy = y2 - y1;
      double segment_length = std::sqrt(dx * dx + dy * dy);
  
      // Normal vector
      double nx = -dy / segment_length;
      double ny = dx / segment_length;
  
      // Calculate curvature by comparing the angle between consecutive segments
      double curvature = 1.0; // Default curvature
      if (i > 0) {
          double x0 = track_points[i - 1].first;
          double y0 = track_points[i - 1].second;
          double dx1 = x1 - x0;
          double dy1 = y1 - y0;
          double segment_length1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
  
          double dot_product = (dx1 * dx + dy1 * dy) / (segment_length1 * segment_length);
          double angle = std::acos(dot_product);
          curvature = std::abs(angle); // Higher angle means higher curvature
      }
  
      // Adjust cone spacing based on curvature
      double adjusted_cone_spacing = cone_spacing / (1.0 + curvature*15);
  
      accumulated_distance += segment_length;
  
      // Place cones at regular intervals
      while (accumulated_distance >= adjusted_cone_spacing) {
          double t = adjusted_cone_spacing / accumulated_distance;
          double cx = x1 + t * dx;
          double cy = y1 + t * dy;
  
          cone_points.emplace_back(cx + cone_distance * nx, cy + cone_distance * ny, 1); // Inner cone
          cone_points.emplace_back(cx - cone_distance * nx, cy - cone_distance * ny, 2); // Outer cone
  
          accumulated_distance -= adjusted_cone_spacing;
      }
  }

  // Generate noise points outside the track
  std::default_random_engine generator(42); // Seed for reproducibility
  std::uniform_real_distribution<double> dist_angle(0, 2 * M_PI);
  std::uniform_real_distribution<double> dist_distance(cone_distance, max_noise_distance);

  for (int i = 0; i < num_noise_points; ++i) {
    bool valid_point = false;
    double noise_x, noise_y;

    while (!valid_point) {
      double noise_distance = dist_distance(generator);
      double noise_angle = dist_angle(generator);
      noise_x = noise_distance * cos(noise_angle);
      noise_y = noise_distance * sin(noise_angle);

      // Find a random point on the track to base the noise point on
      int idx = generator() % track_points.size();
      double base_x = track_points[idx].first;
      double base_y = track_points[idx].second;

      double candidate_x = base_x + noise_x;
      double candidate_y = base_y + noise_y;

      // Check if the candidate point is outside the track
      valid_point = true;
      for (const auto& point : track_points) {
        double dist = std::sqrt(std::pow(candidate_x - point.first, 2) + std::pow(candidate_y - point.second, 2));
        if (dist < cone_distance) {
          valid_point = false;
          break;
        }
      }

      if (valid_point) {
        cone_points.emplace_back(candidate_x, candidate_y, 0); // Noise points with intensity 0
      }
    }
  }

  return cone_points;
}

// Function to generate centerline points of the racetrack
std::vector<std::pair<double, double>> generateCenterlinePoints(const std::vector<std::pair<double, double>>& track_points) {
  std::vector<std::pair<double, double>> centerline_points;
  for (size_t i = 0; i < track_points.size() - 1; ++i) {
    double x1 = track_points[i].first;
    double y1 = track_points[i].second;
    double x2 = track_points[i + 1].first;
    double y2 = track_points[i + 1].second;

    // Midpoint between two points
    double mx = (x1 + x2) / 2;
    double my = (y1 + y2) / 2;

    centerline_points.emplace_back(mx, my);
    
  }
  return centerline_points;
}

// Function to convert frequency in Hz to period in milliseconds
int hz_to_ms(double hz) {
  return static_cast<int>(1000.0 / hz);
}

class LidarConeSim : public rclcpp::Node {
    // Parameters
    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters){
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";
        for (const auto &param : parameters)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "Param update: " << param.get_name().c_str() << ": " << param.value_to_string().c_str());
            if (param.get_name() == "track_keypoints")
            {
                track_keypoints_ = param.as_int();
            }
            if (param.get_name() == "track_radius")
            {
                track_radius_ = param.as_int();
            }
            if (param.get_name() == "track_distortion")
            {
                track_distortion_ = param.as_double();
            }
            if (param.get_name() == "num_interpolated_points")
            {
                num_interpolated_points_ = param.as_int();
            }
            if (param.get_name() == "cone_spacing")
            {
                cone_spacing_ = param.as_double();
            }
            if (param.get_name() == "cone_distance")
            {
                cone_distance_ = param.as_double();
            }
            if (param.get_name() == "update_frequency")
            {
                update_frequency_ = param.as_int();
            }
            if (param.get_name() == "lidar_frame")
            {
                lidar_frame_ = param.as_string();
            }
            if (param.get_name() == "track_pointcloud_topic")
            {
                track_pointcloud_topic_ = param.as_string();
            }
            if (param.get_name() == "centerline_topic")
            {
                centerline_topic_ = param.as_string();
            }
            if (param.get_name() == "seed")
            {
                seed_ = param.as_int();
            }
            if (param.get_name() == "crop_minX")
            {
                crop_minX_ = param.as_double();
            }
            if (param.get_name() == "crop_minY")
            {
                crop_minY_ = param.as_double();
            }
            if (param.get_name() == "crop_maxX")
            {
                crop_maxX_ = param.as_double();
            }
            if (param.get_name() == "crop_maxY")
            {
                crop_maxY_ = param.as_double();
            }
            if (param.get_name() == "visible_pointcloud_topic")
            {
                visible_pointcloud_topic_ = param.as_string();
            }
            if (param.get_name() == "noise_num_points")
            {
                noise_num_points_ = param.as_int();
            }
            if (param.get_name() == "noise_radius")
            {
                noise_radius_ = param.as_int();
            }
        }
        return result;
    }

public:
  LidarConeSim() : Node("lidar_cone_sim"), current_index_(0) {
    this->declare_parameter("track_keypoints", track_keypoints_);
    this->declare_parameter("track_radius", track_radius_);
    this->declare_parameter("track_distortion", track_distortion_);
    this->declare_parameter("num_interpolated_points", num_interpolated_points_);
    this->declare_parameter("cone_spacing", cone_spacing_);
    this->declare_parameter("cone_distance", cone_distance_);
    this->declare_parameter("update_frequency", update_frequency_);
    this->declare_parameter("lidar_frame", lidar_frame_);
    this->declare_parameter("track_pointcloud_topic", track_pointcloud_topic_);
    this->declare_parameter("centerline_topic", centerline_topic_);
    this->declare_parameter("seed", seed_);
    this->declare_parameter("crop_minX", crop_minX_);
    this->declare_parameter("crop_minY", crop_minY_);
    this->declare_parameter("crop_maxX", crop_maxX_);
    this->declare_parameter("crop_maxY", crop_maxY_);
    this->declare_parameter("visible_pointcloud_topic", visible_pointcloud_topic_);
    this->declare_parameter("noise_num_points", noise_num_points_);
    this->declare_parameter("noise_radius", noise_radius_);

    this->get_parameter("track_keypoints", track_keypoints_);
    this->get_parameter("track_radius", track_radius_);
    this->get_parameter("track_distortion", track_distortion_);
    this->get_parameter("num_interpolated_points", num_interpolated_points_);
    this->get_parameter("cone_spacing", cone_spacing_);
    this->get_parameter("cone_distance", cone_distance_);
    this->get_parameter("update_frequency", update_frequency_);
    this->get_parameter("lidar_frame", lidar_frame_);
    this->get_parameter("track_pointcloud_topic", track_pointcloud_topic_);
    this->get_parameter("centerline_topic", centerline_topic_);
    this->get_parameter("seed", seed_);
    this->get_parameter("crop_minX", crop_minX_);
    this->get_parameter("crop_minY", crop_minY_);
    this->get_parameter("crop_maxX", crop_maxX_);
    this->get_parameter("crop_maxY", crop_maxY_);
    this->get_parameter("visible_pointcloud_topic", visible_pointcloud_topic_);
    this->get_parameter("noise_num_points", noise_num_points_);
    this->get_parameter("noise_radius", noise_radius_);

    // if seed is 0, generate a random integer
    if (seed_ == 0) {
      std::random_device rd;
      seed_ = rd();
    }

    pub_lidar = this->create_publisher<sensor_msgs::msg::PointCloud2>(track_pointcloud_topic_, 10);
    vis_lidar = this->create_publisher<sensor_msgs::msg::PointCloud2>(visible_pointcloud_topic_, 10);
    pub_centerline = this->create_publisher<visualization_msgs::msg::MarkerArray>(centerline_topic_, 10);
    callback_handle_ = this->add_on_set_parameters_callback(std::bind(&LidarConeSim::parametersCallback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(hz_to_ms(update_frequency_)),
        std::bind(&LidarConeSim::publish_points, this));

    // Generate racetrack points
    track_points_ = generateDistortedCirclePoints(track_keypoints_, track_radius_, track_distortion_, seed_); // Reduce distortion for smoother curves
    track_points_ = interpolateSpline(track_points_, num_interpolated_points_); // Interpolate with 50 points for smoothness
    cones_ = generateConePoints(track_points_, cone_spacing_, cone_distance_, noise_num_points_, noise_radius_);  // Cone spacing and distance from track
    centerline_points_ = generateCenterlinePoints(track_points_); // Centerline points for visualization
  }

private:
  // Function to publish the lidar point cloud and centerline
  void publish_points() {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    cloud->header.frame_id = lidar_frame_;
    cloud->height = 1;

    // Include both cone points and centerline points in the point cloud
    cloud->width = cones_.size();
    cloud->is_dense = false;
    cloud->points.resize(cloud->width * cloud->height);

    // Calculate the angle of the current segment for reversing direction
    double x1 = track_points_[current_index_].first;
    double y1 = track_points_[current_index_].second;
    double x2 = track_points_[(current_index_ + 1) % track_points_.size()].first;
    double y2 = track_points_[(current_index_ + 1) % track_points_.size()].second;
    double angle = atan2(y1 - y2, x1 - x2); // Reverse direction

    // Function to rotate a point around the origin by -angle and then 90 degrees around y-axis
    auto rotatePoint = [angle](double x, double y) {
        double cos_angle = cos(-angle);
        double sin_angle = sin(-angle);
        double new_x = x * cos_angle - y * sin_angle;
        double new_y = x * sin_angle + y * cos_angle;
        // Rotate 90 degrees around y-axis (z becomes x, x becomes -z)
        double rotated_x = -new_y;
        double rotated_y = new_x;
        double rotated_z = 0.0;
        return std::make_tuple(rotated_x, rotated_y, rotated_z);
    };

    // Update the position of cones relative to the "racecar" at the origin
    for (size_t i = 0; i < cones_.size(); ++i) {
        int idx = (current_index_ + i) % cones_.size();
        auto [new_x, new_y, new_z] = rotatePoint(std::get<0>(cones_[idx]) - x1, std::get<1>(cones_[idx]) - y1);
        cloud->points[i].x = new_x;
        cloud->points[i].y = -new_y;
        cloud->points[i].z = new_z;
        cloud->points[i].intensity = std::get<2>(cones_[idx]); // Set intensity based on inner (1) or outer (2) cone
    }

    // Update the current index
    current_index_ = (current_index_ + 1) % track_points_.size();  // Move to the next track point

    // Publish the lidar point cloud
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*cloud, output_msg);
    output_msg.header.stamp = this->now();
    output_msg.is_dense = true;
    pub_lidar->publish(output_msg);

    // Publish a second pointcloud, with cropped points to simulate "visible" cones
    // Make copy of cloud, as PointXYZ
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);

    // Convert pcl::PointXYZI to pcl::PointXYZ
    for (const auto& pointI : *cloud) {
        pcl::PointXYZ point;
        point.x = pointI.x;
        point.y = pointI.y;
        point.z = pointI.z;
        cloud_xyz->push_back(point);
    }

    // Create cloud to store the cropped points
    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
    cropped_pointcloud->header.frame_id = lidar_frame_;

    pcl::CropBox<pcl::PointXYZ> crop;
    crop.setInputCloud(cloud_xyz);
    crop.setMin(Eigen::Vector4f(crop_minX_, crop_minY_, -5.0, 1.0));
    crop.setMax(Eigen::Vector4f(crop_maxX_, crop_maxY_, 5.0, 1.0));
    crop.filter(*cropped_pointcloud);

    // Publish in vis_lidar
    sensor_msgs::msg::PointCloud2 output_msg_cropped;
    pcl::toROSMsg(*cropped_pointcloud, output_msg_cropped);
    output_msg_cropped.header.stamp = this->now();
    output_msg_cropped.header.frame_id = lidar_frame_;
    output_msg_cropped.is_dense = true;
    vis_lidar->publish(output_msg_cropped);

    // Publish the centerline as a MarkerArray
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker line_strip;
    line_strip.header.frame_id = "laser_data_frame";
    line_strip.header.stamp = this->now();
    line_strip.ns = "racetrack";
    line_strip.id = 0;
    line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_strip.action = visualization_msgs::msg::Marker::ADD;
    line_strip.scale.x = 0.1;  // Line width
    line_strip.color.r = 1.0;
    line_strip.color.g = 0.0;
    line_strip.color.b = 0.0;
    line_strip.color.a = 1.0;
    line_strip.pose.position.z = 1.0;

    // Add the centerline points to the line strip marker
    for (const auto& point : centerline_points_) {
        auto [new_x, new_y, new_z] = rotatePoint(point.first - x1, point.second - y1);
        geometry_msgs::msg::Point p;
        p.x = new_x;
        p.y = -new_y;
        p.z = new_z;
        line_strip.points.push_back(p);
    }

    // Draw a line from the last point to the first point
    auto [first_x, first_y, first_z] = rotatePoint(centerline_points_[0].first - x1, centerline_points_[0].second - y1);
    geometry_msgs::msg::Point p;
    p.x = first_x;
    p.y = -first_y;
    p.z = first_z;
    line_strip.points.push_back(p);

    // Publish the centerline marker
    marker_array.markers.push_back(line_strip);
    pub_centerline->publish(marker_array);
  }

  // ROS2 publishers and timer
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_lidar;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr vis_lidar;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_centerline;
  rclcpp::TimerBase::SharedPtr timer_;
  
  OnSetParametersCallbackHandle::SharedPtr callback_handle_;

  std::vector<std::pair<double, double>> track_points_;
  std::vector<std::tuple<double, double, int>> cones_;
  std::vector<std::pair<double, double>> centerline_points_;
  size_t current_index_;

  // Parameters
  int track_keypoints_ = 20;
  int track_radius_ = 40;
  double track_distortion_ = 0.2;
  int num_interpolated_points_ = 50;
  double cone_spacing_ = 5.0;
  double cone_distance_ = 1.5;
  int update_frequency_ = 20;
  std::string lidar_frame_ = "laser_data_frame";
  std::string track_pointcloud_topic_ = "nonground";
  std::string centerline_topic_ = "racetrack_centerline";
  std::string visible_pointcloud_topic_ = "visible_points";
  int seed_ = 0;
  int noise_num_points_ = 400;
  int noise_radius_ = 10;

  double crop_minX_ = -15.0;
  double crop_minY_ = -15.0;
  double crop_maxX_ = 15.0;
  double crop_maxY_ = 15.0;
};

int main(int argc, char *argv[]) {
  std::cout << "Starting LidarConeSim node..." << std::endl;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarConeSim>());
  rclcpp::shutdown();
  return 0;
}
