#include <iostream>
#include <vector>
#include <cmath>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/msg/string.hpp"


// Take in a markerarray for the predicted points, and a markerarray for the racetrack centerline and std::cout the current the accuracy of the predicted points (considering all previous points)
class AccuracyMeter : public rclcpp::Node{
    // Parameters
    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters){
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";
        for (const auto &param : parameters)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "Param update: " << param.get_name().c_str() << ": " << param.value_to_string().c_str());
            if (param.get_name() == "centerline_topic")
            {
                centerline_topic = param.as_string();
            }
            if (param.get_name() == "verbose")
            {
                verbose = param.as_bool();
            }
            if (param.get_name() == "predicted_points_topic")
            {
                predicted_points_topic = param.as_string();
            }
            if (param.get_name() == "overlay_text_topic")
            {
                overlay_text_topic = param.as_string();
            }
            if (param.get_name() == "accuracy_treshold")
            {
                accuracy_treshold = param.as_double();
            }
        }
        return result;
    }

public:
    AccuracyMeter() : Node("accuracy_meter"){
        this->declare_parameter("centerline_topic", centerline_topic);
        this->declare_parameter("predicted_points_topic", predicted_points_topic);
        this->declare_parameter("verbose", verbose);
        this->declare_parameter("overlay_text_topic", overlay_text_topic);
        this->declare_parameter("accuracy_treshold", accuracy_treshold);

        this->get_parameter("centerline_topic", centerline_topic);
        this->get_parameter("predicted_points_topic", predicted_points_topic);
        this->get_parameter("verbose", verbose);
        this->get_parameter("overlay_text_topic", overlay_text_topic);
        this->get_parameter("accuracy_treshold", accuracy_treshold);

        predicted_points_sub = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            predicted_points_topic, 10, std::bind(&AccuracyMeter::predicted_points_callback, this, std::placeholders::_1));
        centerline_sub = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            centerline_topic, 10, std::bind(&AccuracyMeter::centerline_callback, this, std::placeholders::_1));
        overlay_text_pub = this->create_publisher<std_msgs::msg::String>(overlay_text_topic, 10);
        callback_handle_ = this->add_on_set_parameters_callback(std::bind(&AccuracyMeter::parametersCallback, this, std::placeholders::_1));
    }

private:
void predicted_points_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg){
    if (verbose){
        RCLCPP_INFO(this->get_logger(), "\nPredicted points received");
    }
    predicted_msg = *msg;

    // empty the predicted points
    predicted_points.clear();

    // Initialize a container to hold the points of the line marker
    std::vector<geometry_msgs::msg::Point> line_points;

    // Iterate through the markers to find the LINE_LIST marker
    for (const auto& marker : predicted_msg.markers) {
        if (marker.type == visualization_msgs::msg::Marker::LINE_LIST) {
            // Copy the points from the LINE_LIST marker
            line_points = marker.points;
            break;
        }
    }

    if (line_points.empty()) {
        RCLCPP_WARN(this->get_logger(), "\nNo LINE_LIST marker found in predicted points");
        return;
    }

    // we only need the even points, as line has a starting and ending point for each line
    for (std::vector<geometry_msgs::msg::Point>::size_type i = 0; i < line_points.size(); i += 2) {
        pcl::PointXY point;
        point.x = line_points[i].x;
        point.y = line_points[i].y;

        predicted_points.push_back(point);
    }

    // add the last point if it exists
    if (line_points.size() > 1) {
        pcl::PointXY lPoint;
        lPoint.x = line_points[line_points.size() - 1].x;
        lPoint.y = line_points[line_points.size() - 1].y;
        predicted_points.push_back(lPoint);
    }

    if (verbose){
        RCLCPP_INFO(this->get_logger(), "\nNumber of points in the line marker: %ld", predicted_points.size());
    }

    // if centerline has been received, calculate the accuracy
    if(centerline.size() > 0){
        calculate_accuracy();
    } else {
        RCLCPP_WARN(this->get_logger(), "\nCenterline not received yet");
    }
}

void centerline_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg){
    if (verbose){
        RCLCPP_INFO(this->get_logger(), "\nCenterline received");
    }
    centerline_msg = *msg;
    centerline.clear();

    // Assuming there's only one LINE_STRIP marker in the array that represents the centerline
    for (const auto& marker : centerline_msg.markers) {
        if (marker.type == visualization_msgs::msg::Marker::LINE_STRIP) {
            for (const auto& pt : marker.points) {
                pcl::PointXY point;
                point.x = pt.x;
                point.y = pt.y;
                centerline.push_back(point);
            }
            break; 
        }
    }

    if (centerline.empty()) {
        RCLCPP_ERROR(this->get_logger(), "\nNo LINE_STRIP marker found in centerline");
        return;
    }

    // if predicted points have been received, calculate the accuracy
    if(predicted_points.size() > 0){
        calculate_accuracy();
    } else {
        RCLCPP_WARN(this->get_logger(), "\nPredicted points not received yet");
    }
}

void calculate_accuracy(){
    if (predicted_points.size() < 2) {
        RCLCPP_ERROR(this->get_logger(), "\nNot enough predicted points to calculate accuracy");
        return;
    }

    if (centerline.empty()) {
        RCLCPP_ERROR(this->get_logger(), "\nCenterline is empty");
        return;
    }

    // calculate the distance between the first two predicted points
    double distance_between_predicted_points = sqrt(pow(predicted_points[0].x - predicted_points[1].x, 2) + pow(predicted_points[0].y - predicted_points[1].y, 2));
    if (verbose){
        RCLCPP_INFO(this->get_logger(), "\nDistance between first two predicted points: %f", distance_between_predicted_points);
    }

    // calculate the distance from the first point to each predicted point
    std::vector<double> distances_from_first_point;
    for(size_t i = 0; i < predicted_points.size(); i++){
        distances_from_first_point.push_back(i * distance_between_predicted_points);
    }

    // find the point on the centerline that is closest to where the first predicted point should be
    double min_distance = std::numeric_limits<double>::max();
    int min_index = 0;
    for(size_t i = 0; i < centerline.size(); i++){
        double distance = sqrt(pow(predicted_points[0].x - centerline[i].x, 2) + pow(predicted_points[0].y - centerline[i].y, 2));
        if(distance < min_distance){
            min_distance = distance;
            min_index = i;
        }
    }

    if (min_distance == std::numeric_limits<double>::max()) {
        RCLCPP_ERROR(this->get_logger(), "\nNo minimum distance found between predicted points and centerline");
        return;
    }

    // find the points on the centerline that are closest to where the predicted points should be
    std::vector<int> centerline_point_indexes;
    double accumulated_distance = 0;
    for(size_t i = 0; i < distances_from_first_point.size(); i++){
        for (std::vector<double>::size_type j = min_index; j + 1 < std::floor(centerline.size()*1.5); j++){
            j=j%centerline.size();
            double distance = sqrt(pow(centerline[j].x - centerline[j+1].x, 2) + pow(centerline[j].y - centerline[j+1].y, 2));
            if (accumulated_distance + distance < distances_from_first_point[i]){
                accumulated_distance += distance;
            } else {
                centerline_point_indexes.push_back(j);
                accumulated_distance = 0;
                break;
            }
        }
    }

    if (centerline_point_indexes.size() != predicted_points.size()) {
        RCLCPP_WARN(this->get_logger(), "\nNumber of matched centerline points does not match number of predicted points");
    }

    // calculate the distance between the predicted points and the centerline points
    std::vector<double> distances;
    for(std::vector<pcl::PointXY>::size_type i = 0; i < predicted_points.size(); i++){
        if (i >= centerline_point_indexes.size()) {
            RCLCPP_WARN(this->get_logger(), "\nIndex out of bounds when accessing centerline_point_indexes");
            break;
        }
        distances.push_back(sqrt(pow(predicted_points[i].x - centerline[centerline_point_indexes[i]].x, 2) + pow(predicted_points[i].y - centerline[centerline_point_indexes[i]].y, 2)));
    }

    // if a point is more than {accuracy_treshold} away, it's a miss. (default is 1 meter)
    int misses = 0;
    for(std::vector<double>::size_type i = 1; i < distances.size(); i++){ // starting from one because we know the first point is on the centerline
        if(distances[i] > accuracy_treshold){
            misses++;
        }
    }



    // get the percentage of points that are not misses from the total number of points
    double accuracy = 1 - (double(misses) / double(distances.size() - 1));
    historical_accuracy.push_back(accuracy);

    // calculate historical accuracy
    double historical_accuracy_sum = 0;
    for(std::vector<double>::size_type i = 0; i < historical_accuracy.size(); i++){
        historical_accuracy_sum += historical_accuracy[i];
    }
    double historical_accuracy_average = historical_accuracy_sum / historical_accuracy.size();

    // print as percentage
    if (verbose){
        RCLCPP_INFO(this->get_logger(), "\nAccuracy: %f%%   Historical accuracy: %f%%", accuracy * 100, historical_accuracy_average * 100);
    }
    
    // publish the accuracy as a string
    auto text_msg = std_msgs::msg::String();
    text_msg.data = "Accuracy: " + std::to_string(accuracy * 100) + "%   \nHistorical accuracy: " + std::to_string(historical_accuracy_average * 100) + "%";
    overlay_text_pub->publish(text_msg);
}

rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr predicted_points_sub;
rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr centerline_sub;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr overlay_text_pub;

visualization_msgs::msg::MarkerArray predicted_msg;
visualization_msgs::msg::MarkerArray centerline_msg;

std::vector<pcl::PointXY> predicted_points;
std::vector<pcl::PointXY> centerline;

std::vector<double> historical_accuracy;

OnSetParametersCallbackHandle::SharedPtr callback_handle_;

// Parameters
std::string centerline_topic = "racetrack_centerline";
std::string predicted_points_topic = "debug_marker";
bool verbose = false;
std::string overlay_text_topic = "overlay_text";
double accuracy_treshold = 1.0;

};

int main(int argc, char *argv[])
{
    std::cout << "Starting accuracy meter node..." << std::endl;
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AccuracyMeter>());
    rclcpp::shutdown();
    return 0;
}
