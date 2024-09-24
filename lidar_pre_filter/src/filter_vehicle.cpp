#include <chrono>
#include <functional>
#include <memory>
#include <string>
// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
// TF
#include <pcl_ros/transforms.hpp>
#include <geometry_msgs/msg/transform_stamped.h>
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

class FliterVehicle : public rclcpp::Node
{
    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";
        for (const auto &param : parameters)
        {
            // RCLCPP_INFO_STREAM(this->get_logger(), "Param update: " << param.get_name().c_str() << ": " << param.value_to_string().c_str());
            if (param.get_name() == "cloud_in_topic")
            {
                RCLCPP_INFO_STREAM(this->get_logger(), "cloud_in_topic: " << param.get_value<std::string>());
            }
            else if (param.get_name() == "cam_cones_topic")
            {
                RCLCPP_INFO_STREAM(this->get_logger(), "cam_cones_topic: " << param.get_value<std::string>());
            }
            else if (param.get_name() == "output_topic")
            {
                RCLCPP_INFO_STREAM(this->get_logger(), "output_cloud_topic: " << param.get_value<std::string>());
            }
            else if (param.get_name() == "output_frame")
            {
                RCLCPP_INFO_STREAM(this->get_logger(), "output_frame: " << param.get_value<std::string>());
            }
            else if (param.get_name() == "verbose1")
            {
                RCLCPP_INFO_STREAM(this->get_logger(), "verbose1: " << param.get_value<bool>());
                verbose1 = param.get_value<bool>();
            }
            else if (param.get_name() == "verbose2")
            {
                RCLCPP_INFO_STREAM(this->get_logger(), "verbose2: " << param.get_value<bool>());
                verbose2 = param.get_value<bool>();
            }
            else if (param.get_name() == "toggle_boundary_trim")
            {
                RCLCPP_INFO_STREAM(this->get_logger(), "toggle_boundary_trim: " << param.get_value<bool>());
                toggle_boundary_trim = param.get_value<bool>();
            }
            else if (param.get_name() == "toggle_box_filter")
            {
                RCLCPP_INFO_STREAM(this->get_logger(), "toggle_box_filter: " << param.get_value<bool>());
                toggle_box_filter = param.get_value<bool>();
            }
            else if (param.get_name() == "toggle_cam_filter")
            {
                RCLCPP_INFO_STREAM(this->get_logger(), "toggle_cam_filter: " << param.get_value<bool>());
                toggle_cam_filter = param.get_value<bool>();
            }
            else if (param.get_name() == "crop_boundary")
            {
                crop_boundary = param.get_value<std::vector<double>>();
                printcfg_b();
            }
            else if (param.get_name() == "crop_box_array")
            {
                crop_box_array = param.get_value<std::vector<double>>();
                printcfg_a();
            }
            else if (param.get_name() == "cam_cone_radius")
            {
                RCLCPP_INFO_STREAM(this->get_logger(), "cam_cone_radius: " << param.get_value<float>());
                cam_cone_radius = param.get_value<float>();
            }
            else
            {
                result.successful = false;
                result.reason = "failed";
            }
        }
        return result;
    }

public:
    FliterVehicle() : Node("filter_vehicle"), count_(0)
    {
        // parameters
        this->declare_parameter<std::string>("cloud_in_topic", "input_cloud");
        this->declare_parameter<std::string>("cam_cones_topic", "input_cones");
        this->declare_parameter<std::string>("output_topic", "cloud_prefiltered");
        this->declare_parameter<std::string>("output_frame", "base_link");
        this->declare_parameter<bool>("verbose1", false);
        this->declare_parameter<bool>("verbose2", true);
        this->declare_parameter<bool>("toggle_boundary_trim", true);
        this->declare_parameter<bool>("toggle_box_filter", true);
        this->declare_parameter<bool>("toggle_cam_filter", true);
        this->declare_parameter<std::vector<double>>("crop_boundary",
            std::vector<double>{-200.0, -25.0, -2.0, 200.0, 25.0, 2.0});
        this->declare_parameter<std::vector<double>>("crop_box_array",
            std::vector<double>{-1.2, -1.2, -1.2, 1.2, 1.2, 1.2});
        this->declare_parameter<float>("cam_cone_radius", 0.5);

        this->get_parameter("cloud_in_topic", cloud_in_topic);
        this->get_parameter("cam_cones_topic", cam_cones_topic);
        this->get_parameter("output_topic", output_topic);
        this->get_parameter("output_frame", output_frame);
        this->get_parameter("verbose1", verbose1);
        this->get_parameter("verbose2", verbose2);
        this->get_parameter("toggle_boundary_trim", toggle_boundary_trim);
        this->get_parameter("toggle_box_filter", toggle_box_filter);
        this->get_parameter("toggle_cam_filter", toggle_cam_filter);
        this->get_parameter("crop_boundary", crop_boundary);
        this->get_parameter("crop_box_array", crop_box_array);
        this->get_parameter("cam_cone_radius", cam_cone_radius);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        pub_lidar_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 10);
        sub_lidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(cloud_in_topic,
            rclcpp::SensorDataQoS().keep_last(1), std::bind(&FliterVehicle::lidar_callback, this, std::placeholders::_1));
        sub_cam_cones_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(cam_cones_topic,
            rclcpp::SensorDataQoS().keep_last(1), std::bind(&FliterVehicle::cam_cones_callback, this, std::placeholders::_1));
        callback_handle_ = this->add_on_set_parameters_callback(std::bind(&FliterVehicle::parametersCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "FliterVehicle node has been started.");
        RCLCPP_INFO_STREAM(this->get_logger(), "cloud_in_topic: " << this->get_parameter("cloud_in_topic").as_string());
        RCLCPP_INFO_STREAM(this->get_logger(), "cam_cones_topic: " << this->get_parameter("cam_cones_topic").as_string());
        RCLCPP_INFO_STREAM(this->get_logger(), "output_topic: " << this->get_parameter("output_topic").as_string());
        RCLCPP_INFO_STREAM(this->get_logger(), "output_frame: " << this->get_parameter("output_frame").as_string());
        printcfg_b();   //show info about pcl boundary cropping ("trimming" of the points too far)
        printcfg_a();   //show info about pcl array cropping (~=shape of the vehicle to be removed)
    }

private:
    void lidar_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg)
    {
        RCLCPP_INFO_ONCE(this->get_logger(), "First PCL input recieved, BoxFilter is turned %s\n"
            "Trimming of the pointcloud at the boundaries is turned %s",
            (toggle_box_filter ? "ON" : "OFF"), (toggle_boundary_trim ? "ON" : "OFF"));
        
        //ROS2 msg - final output msg + temporarily holds input after transformation
        sensor_msgs::msg::PointCloud2 output_msg;
        //pcl version to operate on later
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        //if every module is turned off: output = input (transformed)
        pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud = cloud;
        // TF
        if (input_msg->header.frame_id != output_frame)
        {
            try
            {
                tf = tf_buffer_->lookupTransform(output_frame, input_msg->header.frame_id, tf2::TimePointZero);
                pcl_ros::transformPointCloud(output_frame, tf, *input_msg, output_msg);
            }
            catch (const tf2::TransformException & ex)
            {
                RCLCPP_ERROR_ONCE(
                    this->get_logger(), "[PCL TF] Could not transform %s to %s: %s",
                    input_msg->header.frame_id.c_str(), output_frame.c_str(), ex.what());
                return;
            }
            pcl::fromROSMsg(output_msg, *cloud);    //output_msg is the transformed input at this point
        }
        else pcl::fromROSMsg(*input_msg, *cloud);   //let pcl through without transformation

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_trimmed(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cropped(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cones(new pcl::PointCloud<pcl::PointXYZI>);
        
        if (toggle_boundary_trim)
        {
            // Filter out points outside of the box
            pcl::CropBox<pcl::PointXYZI> crop_over;
            std::vector<float> passvec;
            passvec.resize(crop_boundary.size());
            for (int i = crop_boundary.size() - 1; i >= 0; i--) passvec[i] = crop_boundary[i];
            if (passvec.size() == 6)            //sanity check
            {
                crop_over.setInputCloud(cloud);
                crop_over.setMin(Eigen::Vector4f(passvec[0], passvec[1], passvec[2], 1.0));
                crop_over.setMax(Eigen::Vector4f(passvec[3], passvec[4], passvec[5], 1.0));
                crop_over.filter(*cloud_trimmed);
                output_cloud = cloud_trimmed;
            }
            else RCLCPP_ERROR(this->get_logger(), "[Boundary Filter] ERROR: 6 parameters expected!"
                " (minX, minY, minZ, maxX, maxY, maxZ <-> %ld recieved)", passvec.size());
        }
        if (toggle_box_filter)
        {
            // Array-based crop box! (multiple boxes, sequential filtering)
            pcl::CropBox<pcl::PointXYZI>* crop_box;
            std::vector<float> passvec;
            passvec.resize(crop_box_array.size());
            for (int i = crop_box_array.size() - 1; i >= 0; i--) passvec[i] = crop_box_array[i];
            if (!(passvec.size() % 6))          //sanity check
            {
                int s = crop_box_array.size();
                for (int i = 0; i < s; i += 6)  //per crop box
                {
                    crop_box = new pcl::CropBox<pcl::PointXYZI>;
                    if (!i)                                             //first time
                    {
                        if (toggle_boundary_trim)
                            crop_box->setInputCloud(cloud_trimmed);     //trimmed input cloud
                        else crop_box->setInputCloud(cloud);            //original input cloud
                    }
                    else crop_box->setInputCloud(cloud_cropped);        //repeat on previous result
                    crop_box->setMin(Eigen::Vector4f(passvec[i], passvec[i+1], passvec[i+2], 1.0));
                    crop_box->setMax(Eigen::Vector4f(passvec[i+3], passvec[i+4], passvec[i+5], 1.0));
                    crop_box->setNegative(true);
                    crop_box->filter(*cloud_cropped);
                    delete crop_box;
                }
                output_cloud = cloud_cropped;
            }
            else RCLCPP_ERROR(this->get_logger(),
                "[Array Filter] %s [%ld recieved]", ERROR_TEXT_PARAM_NUM, passvec.size());
        }
        if (toggle_cam_filter)
        {
            int s = output_cloud->points.size();
            int c = cones.size();
            float x, y, r,
                rs = cam_cone_radius * cam_cone_radius; //acceptable radius (from a cone), squared
            for (int i = 0; i < s; i++)     //for every point
            {
                for (int j = 0; j < c; j++) //for every cone
                {
                    x = cones[j].x - output_cloud->points[i].x;    //delta x
                    y = cones[j].y - output_cloud->points[i].y;    //delta y
                    r = x * x + y * y;  //squared distance
                    if (r < rs)         //no need for sqrt() for comparison
                    {
                        cloud_cones->points.push_back(output_cloud->points[i]);
                        break;  //avoid duplicates (and unnecessary computations)
                    }
                }
            }
            output_cloud = cloud_cones;
        }
        pcl::toROSMsg(*output_cloud, output_msg);   //output_msg is the final output msg now
        // Add the same frame_id as the input, it is not included in pcl PointXYZI
        output_msg.header.frame_id = output_frame;
        pub_lidar_->publish(output_msg);    // Publish the cloud data as a ROS message
    }
    void cam_cones_callback(const visualization_msgs::msg::MarkerArray::ConstSharedPtr markers_in)
    {
        RCLCPP_INFO_ONCE(this->get_logger(), "First CamCone input recieved, CamFilter is turned %s",
            (toggle_cam_filter ? "ON" : "OFF"));
        if (!toggle_cam_filter) return;
        int s = markers_in->markers.size();
        /*  //TF (unused)
        double tfx = 0; //x component of translation
        double tfy = 0; //y component of translation
        double tfs = 0; //sine component of rotation
        double tfc = 1; //cosine component of rotation
        */
        if (s)
        {
            if (markers_in->markers[0].header.frame_id != output_frame)
            {
                RCLCPP_ERROR_ONCE(this->get_logger(),
                    "[Cam TF] Marker frame does not match output frame! (%s)\n",
                    output_frame.c_str());
                //return;
                /*  //TF (unused)
                try
                {
                    tf = tf_buffer_->lookupTransform(output_frame, markers_in->markers[0].header.frame_id, tf2::TimePointZero);
                    tfx = tf.transform.translation.x;
                    tfy = tf.transform.translation.y;
                    geometry_msgs::msg::Quaternion q = tf.transform.rotation;
                    //some 3D->2D quaternion magic
                    tfs = 2 * (q.w * q.z + q.x * q.y);      //sine componenet of rot matrix coeff
                    tfc = 1 - 2 * (q.y * q.y + q.z * q.z);  //cosine component od rot matrix coeff
                }
                catch (const tf2::TransformException & ex)
                {
                    RCLCPP_ERROR_ONCE(
                        this->get_logger(), "[Cam TF] Could not transform %s to %s: %s"
                        "Turning CamFilter OFF! (can be turned back on via param)",
                        markers_in->markers[0].header.frame_id.c_str(), output_frame.c_str(), ex.what());
                        //toggle_cam_filter = false;
                        //this->set_parameter(rclcpp::Parameter("toggle_cam_filter", false));
                    return;
                }
                */
            }
        }
        cones.clear();
        pcl::PointXY tempoint;
        float eps = 0.001;  //smallest number (+-) to not be considered zero
        for (int i = 0; i < s; i++)
        {
            tempoint.x = markers_in->markers[i].pose.position.x;
            tempoint.y = markers_in->markers[i].pose.position.y;
            if ((tempoint.x > -eps && tempoint.x < +eps) &&
                (tempoint.y > -eps && tempoint.y < +eps))
                continue;   //if ~zero: ignore
            /*  //TF (unused)
            //translation + rotation (rot matrix coefficients)
            tempoint.y = tfy + ( tfs * tempoint.x + tfc * tempoint.y );
            tempoint.x = tfx + ( tfc * tempoint.x - tfs * markers_in->markers[i].pose.position.y ); 
            //                                            ^ (tempoint.y got overwritten)
            */
            cones.push_back(tempoint);
        }
    }
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_lidar_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sub_cam_cones_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;
    std::string cloud_in_topic = "pointcloud_topic";
    std::string cam_cones_topic = "cone_coordinates";
    std::string output_topic = "cloud_prefiltered";
    std::string output_frame = "base_link";

    geometry_msgs::msg::TransformStamped tf;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    std::vector<double> crop_boundary = {-10.0, -5.0, -2.0, 10.0, 5.0, 2.0};
    std::vector<double> crop_box_array = {-0.8, -0.8, -0.8, 0.8, 0.8, 0.8};
    float cam_cone_radius = 0.5;
    bool verbose1 = false;
    bool verbose2 = true;
    bool toggle_boundary_trim = true;
    bool toggle_box_filter = true;
    bool toggle_cam_filter = true;
    size_t count_;
    const char* ERROR_TEXT_PARAM_NUM = "ERROR: Incorrect number of Crop Box parameters!"
                " (should be a multiple of 6: minX, minY, minZ, maxX, maxY, maxZ)";

    void printcfg_a()   //print array filter params
    {
        int s = crop_box_array.size();
        if (s % 6)
            RCLCPP_ERROR(this->get_logger(),
            "[Array Filter] %s [%d recieved]", ERROR_TEXT_PARAM_NUM, s);
        else
        {   
            std::string tempstr = "Crop Box Array Filter parameters:"
                "\n#\t minX\t minY\t minZ\t maxX\t maxY\t maxZ";
            char* buff = new char[52];  //line length is 52 in this format
            for (int i = 0; i < s; i += 6)
            {
                snprintf(buff, 52,
                    "\n%d\t%+5.2f\t%+5.2f\t%+5.2f\t%+5.2f\t%+5.2f\t%+5.2f",
                    i / 6,
                    crop_box_array[i],      //minX
                    crop_box_array[i+1],    //minY
                    crop_box_array[i+2],    //minZ
                    crop_box_array[i+3],    //maxX
                    crop_box_array[i+4],    //maxY
                    crop_box_array[i+5]);   //maxZ
                    tempstr += buff;
            }
            tempstr += "\n-----";
            RCLCPP_INFO_STREAM(this->get_logger(), tempstr);
            delete[] buff;
        }
    }
    void printcfg_b()   //print boundary filter params
    {
        int s = crop_boundary.size();
        if (s != 6)
            RCLCPP_ERROR(this->get_logger(), "ERROR: 6 parameters expected!"
                "(minX, minY, minZ, maxX, maxY, maxZ <-> %d recieved)", s);
        else
        {
            std::string tempstr = "Pointcloud boundaries:\n[Crop Boundary Filter parameters]";
            char* buff = new char[76];      //needed string length is 76 in this format
            snprintf(buff, 76,
                "\nX:\t%+8.2f\t-->\t%+8.2f"
                "\nY:\t%+8.2f\t-->\t%+8.2f"
                "\nZ:\t%+8.2f\t-->\t%+8.2f",
                crop_boundary[0],   //minX
                crop_boundary[3],   //maxX
                crop_boundary[1],   //minY
                crop_boundary[4],   //maxY
                crop_boundary[2],   //minZ
                crop_boundary[5]);  //maxZ
            tempstr += buff;
            RCLCPP_INFO_STREAM(this->get_logger(), tempstr);
            delete[] buff;
        }
    }
    std::vector<pcl::PointXY> cones;    //2D coordinates of cones (preprocessed)
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FliterVehicle>());
    rclcpp::shutdown();
    return 0;
}