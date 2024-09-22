#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <unordered_map>
#include <set>

using std::placeholders::_1;

class Deproject {
public:
    Deproject(double roll_deg = 0.0, double pitch_deg = -2.0, double yaw_deg = 0.0, double height = 1.602,
              std::vector<double> K_vec = {335.20611572265625, 0.0, 331.85467529296875, 0.0, 335.20611572265625, 183.79928588867188, 0.0, 0.0, 1.0},
              std::vector<double> R_vec = {M_PI / 2, -M_PI / 2, 0})
        : h_(height), K_vec_(K_vec), R_vec_(R_vec) {
        roll_ = roll_deg * M_PI / 180;
        pitch_ = pitch_deg * M_PI / 180;
        yaw_ = yaw_deg * M_PI / 180;

        calculate_transforms(R_vec_);
        calculate_intrinsics(K_vec_);
    }

    Eigen::Vector3d run(double u, double v) {
        Eigen::Vector3d vector = deproject_pixel(u, v);
        vector = transform_cam_to_road(vector);
        vector = transform_road_to_ground_link(vector);
        return vector;
    }

private:
    double h_;
    double roll_, pitch_, yaw_;
    Eigen::Matrix3d R_cr_, R_rc_, R_gr_;
    Eigen::Vector3d t_rc_;
    Eigen::Matrix3d K_, K_inv_;
    std::vector<double> K_vec_, R_vec_;

    void calculate_transforms(const std::vector<double>& R) {
        double cy = std::cos(yaw_), sy = std::sin(yaw_);
        double cp = std::cos(pitch_), sp = std::sin(pitch_);
        double cr = std::cos(roll_), sr = std::sin(roll_);

        R_cr_ << cr * cy + sp * sr * sy, cr * sp * sy - cy * sr, -cp * sy,
                 cp * sr,                cp * cr,                sp,
                 cr * sy - cy * sp * sr, -cr * cy * sp - sr * sy, cp * cy;

        R_rc_ = R_cr_.transpose();
        t_rc_ = Eigen::Vector3d(0.06, -h_, 0.0);

        Eigen::Matrix3d R_y_90;
        R_y_90 << 0, 0, 1,
                0, 1, 0,
                -1, 0, 0;

        Eigen::Matrix3d R_z_90;
        R_z_90 << 0, -1, 0,
                1, 0, 0,
                0, 0, 1;

        Eigen::Matrix3d R_combined = R_z_90 * R_y_90;

        Eigen::Matrix3d M_y;
        M_y << -1,  0,  0,
                0,  1,  0,
                0,  0, -1;

        Eigen::Matrix3d M_z;
        M_z << -1,  0,  0,
                0, -1,  0,
                0,  0,  1;

        Eigen::Matrix3d R_gr_temp = (Eigen::AngleAxisd(R[0], Eigen::Vector3d::UnitY()) *
                                 Eigen::AngleAxisd(R[1], Eigen::Vector3d::UnitX()) *
                                 Eigen::AngleAxisd(R[2], Eigen::Vector3d::UnitZ())).toRotationMatrix();

        R_gr_ = ((R_combined * R_gr_temp) * M_y) * M_z;
    }

    void calculate_intrinsics(const std::vector<double>& K) {
        K_ = Eigen::Matrix3d();
        K_ << K[0], K[1], K[2],
              K[3], K[4], K[5],
              K[6], K[7], K[8];
        K_inv_ = K_.inverse();
    }

    Eigen::Vector3d deproject_pixel(double u, double v) {
        Eigen::Vector3d uv_h(u, v, 1.0);
        Eigen::Vector3d n_r(0.0, 1.0, 0.0);

        Eigen::Vector3d K_inv_uv_h = (K_inv_ * uv_h).eval();
        Eigen::Vector3d n_c = (R_cr_ * n_r).eval();
        double denominator = n_c.dot(K_inv_uv_h);

        return h_ * K_inv_uv_h / denominator;
    }

    Eigen::Vector3d transform_cam_to_road(const Eigen::Vector3d& vector) {
        return (R_rc_ * vector + t_rc_).eval();
    }

    Eigen::Vector3d transform_road_to_ground_link(const Eigen::Vector3d& vector) {
        return (R_gr_ * vector).eval();
    }
};

class DeprojectionNode : public rclcpp::Node {
public:
    DeprojectionNode() : Node("deprojection_node") {
        deprojector_ = std::make_shared<Deproject>();
        subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "cone_coordinates", 10, std::bind(&DeprojectionNode::listener_callback, this, _1));
        
        //deproj_cones
        publisher_cones_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("deproj_cones", 10);
        pub_proj_coords_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("pub_proj_coords", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&DeprojectionNode::timer_callback, this));
    }

private:
    std::shared_ptr<Deproject> deprojector_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
    //deproj_cones
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_cones_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_proj_coords_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<double> corr_coeffs = {
        5.0,        10.0,       15.0,       20.0,       25.0,       30.0,       //X
        1.020905,   -1.909484,  -2.275741,  -1.418196,  -0.838905,  0.217884    //dX
    };

    struct MarkerInfo {
        Eigen::Vector3d position;
        rclcpp::Time timestamp;
        int cone_id;  //find cone color
    };

    std::unordered_map<std::string, MarkerInfo> markers_;

    void listener_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() % 5 != 0) {
            return;
        }

        std_msgs::msg::Float32MultiArray pub_msg;
        auto current_time = this->now();

        for (size_t i = 0; i < msg->data.size(); i += 5) {
            int cone_id = static_cast<int>(msg->data[i]);
            double u = msg->data[i + 1];
            double v = msg->data[i + 2];
            if (v == 0.0) {
                continue;
            }

            Eigen::Vector3d point = deprojector_->run(u, v);
            pub_msg.data.push_back(cone_id);
            pub_msg.data.push_back(point[0]);
            pub_msg.data.push_back(point[1]);

            std::string marker_id = std::to_string(u) + "_" + std::to_string(v);
            MarkerInfo marker_info{point, current_time, cone_id};

            markers_[marker_id] = marker_info;  
        }

        pub_proj_coords_->publish(pub_msg);
    }

    void timer_callback() {
        auto current_time = this->now();
        double time_threshold = 0.1; 
        std::unordered_map<std::string, MarkerInfo> new_markers;
        process_markers(markers_, new_markers, current_time, time_threshold);
        publish_markers(new_markers, publisher_cones_);
    }

    void process_markers(std::unordered_map<std::string, MarkerInfo>& markers, 
                         std::unordered_map<std::string, MarkerInfo>& new_markers, 
                         rclcpp::Time current_time, double time_threshold) {
        for (auto it = markers.begin(); it != markers.end(); ++it) {
            if ((current_time - it->second.timestamp).seconds() < time_threshold) {
                new_markers[it->first] = it->second;
            }
        }
        markers.swap(new_markers);
    }

    void publish_markers(const std::unordered_map<std::string, MarkerInfo>& markers,
                         rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher) {
        visualization_msgs::msg::MarkerArray marker_array;
        static int id_counter = 0;
        static std::set<int> active_marker_ids;

        //clear func
        for (int active_id : active_marker_ids) {
            visualization_msgs::msg::Marker delete_marker;
            delete_marker.header.frame_id = "laser_sensor_frame";
            delete_marker.action = visualization_msgs::msg::Marker::DELETE;
            delete_marker.id = active_id;
            marker_array.markers.push_back(delete_marker);
        }
        active_marker_ids.clear();

        //publish new markers
        for (const auto& [marker_id, marker_info] : markers) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "laser_sensor_frame";
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.scale.x = 0.3;
            marker.scale.y = 0.3;
            marker.scale.z = 0.3;
            marker.color.a = 1.0;
            marker.id = id_counter++;

            if (marker_info.cone_id == 1) {
                marker.color.r = 1.0;
                marker.color.g = 1.0;
                marker.color.b = 0.3;
            } else if (marker_info.cone_id == 3) {
                marker.color.r = 1.0;
                marker.color.g = 0.5;
                marker.color.b = 0.1;
            } else if (marker_info.cone_id == 2) {
                marker.color.r = 0.0;
                marker.color.g = 0.0;
                marker.color.b = 1.0;
            }

            marker.pose.position.x = marker_info.position[0];
            marker.pose.position.y = marker_info.position[1];
            marker.pose.position.z = marker_info.position[2];
            //correctpt(marker.pose.position);                    //correct deprojection error
            marker_array.markers.push_back(marker);
            active_marker_ids.insert(marker.id);
        }

        publisher->publish(marker_array);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DeprojectionNode>());
    rclcpp::shutdown();
    return 0;
}
