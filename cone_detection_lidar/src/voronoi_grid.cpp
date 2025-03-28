/*
Pathinfinding in a racetrack environment using a Voronoi diagram.

Steps:
1. Constructs the Voronoi diagram using the visible points.
2. Finds the edges or the voronoi diagram that could be associated with an L/R pair of cones of the racetrack.
3. Find a path between the cones using beam casting.
4. TODO: Find the best possible path between the cones.
5: TODO: More cleanup, maybe full refactor.

@Zahuczky
*/

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
#include "cone_detection_lidar/voronoig.hpp"


    /**
     * @brief Calculates the Euclidean distance between two points.
     * 
     * @param p1 The first point.
     * @param p2 The second point.
     * @return The Euclidean distance between p1 and p2.
     */
    double distance(const Point& p1, const Point& p2) {
        return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
    }

    /**
     * @brief Structure to hold a single line defined by two points.
     */
    struct Line {
        Point p1; ///< The first point of the line.
        Point p2; ///< The second point of the line.

        /**
         * @brief Constructor to initialize a Line with given points.
         * 
         * @param p1 The first point of the line.
         * @param p2 The second point of the line.
         */
        Line(Point p1, Point p2) : p1(p1), p2(p2) {}
    };

    /**
     * @brief Determines the orientation of the ordered triplet (p, q, r).
     * 
     * The function returns:
     * - 0 if p, q, and r are collinear (on the same line).
     * - 1 if the triplet is in a clockwise orientation.
     * - 2 if the triplet is in a counterclockwise orientation.
     * 
     * @param p The first point of the triplet.
     * @param q The second point of the triplet.
     * @param r The third point of the triplet.
     * @return An integer representing the orientation:
     *         - 0 for collinear
     *         - 1 for clockwise
     *         - 2 for counterclockwise
     */
    int orientation(const Point& p, const Point& q, const Point& r) {
        double val = (q.y - p.y) * (r.x - q.x) -
                     (q.x - p.x) * (r.y - q.y);
    
        if (val == 0) return 0; // collinear
        return (val > 0) ? 1 : 2; // clock or counterclockwise
    }

    /**
     * @brief Checks if point q lies on the line segment defined by points p and r.
     * 
     * This function determines whether point q lies on the line segment pr by checking
     * if q's coordinates are within the bounding box defined by p and r.
     * 
     * @param p The first point of the line segment.
     * @param q The point to check.
     * @param r The second point of the line segment.
     * @return True if point q lies on the line segment pr, false otherwise.
     */
    bool onSegment(const Point& p, const Point& q, const Point& r) {
        if (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
            q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y))
            return true;
        return false;
    }

    /**
     * @brief Checks if two line segments intersect.
     * 
     * This function determines whether two line segments, l1 and l2, intersect.
     * It uses the orientation of the points to check for general and special cases
     * of intersection.
     * 
     * @param l1 The first line segment.
     * @param l2 The second line segment.
     * @return True if the line segments intersect, false otherwise.
     */
    bool doIntersect(const Line& l1, const Line& l2) {
        const Point& p1 = l1.p1;
        const Point& q1 = l1.p2;
        const Point& p2 = l2.p1;
        const Point& q2 = l2.p2;
    
        // Find the four orientations needed for general and special cases
        int o1 = orientation(p1, q1, p2);
        int o2 = orientation(p1, q1, q2);
        int o3 = orientation(p2, q2, p1);
        int o4 = orientation(p2, q2, q1);
    
        // General case
        if (o1 != o2 && o3 != o4)
            return true;
    
        // Special Cases
        // p1, q1 and p2 are collinear and p2 lies on segment p1q1
        if (o1 == 0 && onSegment(p1, p2, q1)) return true;
    
        // p1, q1 and q2 are collinear and q2 lies on segment p1q1
        if (o2 == 0 && onSegment(p1, q2, q1)) return true;
    
        // p2, q2 and p1 are collinear and p1 lies on segment p2q2
        if (o3 == 0 && onSegment(p2, p1, q2)) return true;
    
        // p2, q2 and q1 are collinear and q1 lies on segment p2q2
        if (o4 == 0 && onSegment(p2, q1, q2)) return true;
    
        return false; // Doesn't fall in any of the above cases, so no intersection
    }

    /**
     * @brief Calculates the midpoint of two points.
     * 
     * This function returns the midpoint of the line segment defined by points A and B.
     * 
     * @param A The first point.
     * @param B The second point.
     * @return The midpoint of the line segment AB.
     */
    Point midpoint(const Point& A, const Point& B) {
        return {(A.x + B.x) / 2, (A.y + B.y) / 2};
    }

    /**
     * @brief Calculates a point that is a specified distance away from the midpoint of the line segment AB, perpendicular to the line.
     * 
     * This function returns a point that is perpendicular to the line segment defined by points A and B, at a specified distance from the midpoint of the line segment.
     * An optional offset can be applied to the calculated point.
     * 
     * @param A The first point of the line segment.
     * @param B The second point of the line segment.
     * @param distance The distance from the midpoint to the perpendicular point.
     * @param offset An optional offset to apply to the calculated point (default is 0).
     * @return A point that is perpendicular to the line segment AB at the specified distance from the midpoint.
     */
    Point perpendicularPoint(const Point& A, const Point& B, double distance, double offset = 0) {
        Point M = midpoint(A, B);
        double dx = B.x - A.x;
        double dy = B.y - A.y;
    
        if (dx == 0) {
            // The original line is vertical, so the perpendicular line is horizontal
            Point perpPoint = {M.x + distance, M.y};
            if (perpPoint.y <= std::max(A.y, B.y)) {
                perpPoint.x = M.x - distance; // Move to the other side
            }
            perpPoint.x += offset;
            return perpPoint;
        } else if (dy == 0) {
            // The original line is horizontal, so the perpendicular line is vertical
            Point perpPoint = {M.x, M.y + distance};
            if (perpPoint.y <= std::max(A.y, B.y)) {
                perpPoint.y = M.y - distance; // Move to the other side
            }
            perpPoint.y += offset;
            return perpPoint;
        } else {
            // Normalize the direction vector of the line segment
            double length = sqrt(dx * dx + dy * dy);
            dx /= length;
            dy /= length;
    
            // Find the perpendicular direction
            double perp_dx = -dy;
            double perp_dy = dx;
    
            // Calculate the new point at the specified distance
            Point perpPoint = {M.x + perp_dx * distance, M.y + perp_dy * distance};
    
            // Check if the perpendicular point is below the y position of both A and B
            if (perpPoint.y <= std::max(A.y, B.y)) {
                // Move the point to the other side of the line
                perpPoint = {M.x - perp_dx * distance, M.y - perp_dy * distance};
            }
            perpPoint.x += offset;
            return perpPoint;
        }
    }

    /**
     * @brief Converts an angle from degrees to radians.
     * 
     * This function takes an angle in degrees and converts it to radians.
     * 
     * @param degrees The angle in degrees to be converted.
     * @return The angle in radians.
     */
    double degreesToRadians(double degrees) {
        return degrees * M_PI / 180.0;
    }

    /**
     * @brief Rotates a point around an origin by a given angle in degrees.
     * 
     * This function rotates a given point around a specified origin by a given angle in degrees.
     * The rotation is performed in a counterclockwise direction.
     * 
     * @param point The point to be rotated.
     * @param origin The origin around which the point is to be rotated.
     * @param degrees The angle in degrees by which to rotate the point.
     * @return The rotated point.
     */
    Point rotatePoint(const Point& point, const Point& origin, double degrees) {
        // Convert the angle from degrees to radians
        double radians = degreesToRadians(-degrees);
    
        // Translate the point to the origin
        double translatedX = point.x - origin.x;
        double translatedY = point.y - origin.y;
    
        // Rotate the point
        double rotatedX = translatedX * cos(radians) - translatedY * sin(radians);
        double rotatedY = translatedX * sin(radians) + translatedY * cos(radians);
    
        // Translate the point back to its original position
        Point rotatedPoint;
        rotatedPoint.x = rotatedX + origin.x;
        rotatedPoint.y = rotatedY + origin.y;
    
        // Correct for small floating-point errors
        if (std::abs(rotatedPoint.x) < 1e-10) rotatedPoint.x = 0;
        if (std::abs(rotatedPoint.y) < 1e-10) rotatedPoint.y = 0;
    
        return rotatedPoint;
    }



class VoronoiNode : public rclcpp::Node {
public:
    VoronoiNode() : Node("voronoi_node") {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "visible_points", 10, std::bind(&VoronoiNode::pointCloudCallback, this, std::placeholders::_1));

        voronoi_marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("voronoi_edges", 10);
        voronoi_orig_p_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("voronoi_edges_orig", 10);
        pred_point_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("pred_points", 10);
        search_area_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("search_area", 10); 
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // Convert PointCloud2 message to PCL point cloud
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl::fromROSMsg(*msg, pcl_cloud);

        // Convert PCL point cloud to vector of Points for Voronoi diagram
        std::vector<Point> points;
        for (const auto& pt : pcl_cloud.points) {
            points.push_back(Point(pt.x, pt.y));
        }

        // Construct the Voronoi diagram using the points
        vrn::VoronoiDiagram vd(points);

        // // Get the reference to the underlying Voronoi diagram
        // const auto& voronoi = vd.getVoronoiDiagram();

        auto const edges = vd.getVoronoiEdges();
        // make line markerarray from edges
        visualization_msgs::msg::MarkerArray marker_array;
        int index = 0;  // Initialize the loop index
        for (const auto& edge : edges) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "laser_data_frame";
            marker.header.stamp = this->now();
            marker.ns = "voronoi_edges_" + std::to_string(index);  // Append the index to the namespace
            marker.id = index;  // Use the index as the marker ID
            marker.type = visualization_msgs::msg::Marker::LINE_LIST;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.1;
            marker.color.b = 1.0;
            marker.color.a = 0.2;
            geometry_msgs::msg::Point p1, p2;
            p1.x = edge.edgeP1.x;
            p1.y = edge.edgeP1.y;
            p2.x = edge.edgeP2.x;
            p2.y = edge.edgeP2.y;
            marker.points.push_back(p1);
            marker.points.push_back(p2);
            marker_array.markers.push_back(marker);
            ++index;  // Increment the loop index
        }

        // publish original points from edges as marker points
        visualization_msgs::msg::MarkerArray marker_array2;
        int index2 = 0;  // Initialize the loop index
        for (const auto& edge : edges) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "laser_data_frame";
            marker.header.stamp = this->now();
            marker.ns = "voronoi_edges_orig_" + std::to_string(index2);  // Append the index to the namespace
            marker.id = index2;  // Use the index as the marker ID
            marker.type = visualization_msgs::msg::Marker::POINTS;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.4;
            marker.scale.y = 0.4;
            marker.color.g = 1.0;
            marker.color.a = 1.0;
            geometry_msgs::msg::Point p1, p2;
            p1.x = edge.origP1.x;
            p1.y = edge.origP1.y;
            p2.x = edge.origP2.x;
            p2.y = edge.origP2.y;
            marker.points.push_back(p1);
            marker.points.push_back(p2);
            marker_array2.markers.push_back(marker);
            ++index2;  // Increment the loop index
        }

        /****************************************************************
         *               Interesting bits start here                    *
         *                                                              *
        *****************************************************************/


        // if the two original points of an edge are between 2.9 and 3.1 meters apart, consider it to be racetrack line, save a pointer to it
        std::vector<const vrn::VoronoiEdge*> racetrack_edges;
        for (const auto& edge : edges) {
            double distance = sqrt(pow(edge.origP1.x - edge.origP2.x, 2) + pow(edge.origP1.y - edge.origP2.y, 2));
            if (distance > 2.9 && distance < 3.1) {
                // if the edge length is shorter than 6 meter
                double edge_length = sqrt(pow(edge.edgeP1.x - edge.edgeP2.x, 2) + pow(edge.edgeP1.y - edge.edgeP2.y, 2));
                if (edge_length < 6) {
                    racetrack_edges.push_back(&edge);  // Stores a pointer to the original edge
                }
            }
        }

        visualization_msgs::msg::MarkerArray orig_ponts;
        int indexor = 0;  // Initialize the loop index
        for (const auto& edge : racetrack_edges) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "laser_data_frame";
            marker.header.stamp = this->now();
            marker.ns = "racetrack_points_" + std::to_string(indexor);  // Append the index to the namespace
            marker.id = indexor;  // Use the index as the marker ID
            marker.type = visualization_msgs::msg::Marker::POINTS;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.4;
            marker.scale.y = 0.4;
            marker.color.g = 1.0;
            marker.color.a = 1.0;
            geometry_msgs::msg::Point p1, p2;
            p1.x = edge->origP1.x;
            p1.y = edge->origP1.y;
            p2.x = edge->origP2.x;
            p2.y = edge->origP2.y;
            marker.points.push_back(p1);
            marker.points.push_back(p2);
            orig_ponts.markers.push_back(marker);
            ++indexor;  // Increment the loop index
        }

        // find the longest path
        std::vector<Point> predicted_points;

        // create an initial beam from the origin forward
        Line initialBeam(Point(0, 0), Point(0, 6));

        // find the first intersection of the initial beam with the racetrack line
        int first_intersection_index = -1;
        int iterCounter = 0;
        int maxIter = 12;
        double angle = 22.5;
        bool moveRoght = true;

        while (first_intersection_index == -1 && iterCounter < maxIter) {
            for (int i = 0; i < int(racetrack_edges.size()); i++) {
                if (doIntersect(initialBeam, Line(racetrack_edges[i]->origP1, racetrack_edges[i]->origP2))) {
                    first_intersection_index = i;
                    break;
                }
            }

            // if no intersection is found, rotate the beam slightly and try again
            if (first_intersection_index == -1) {
                if (moveRoght) {
                    initialBeam.p2 = rotatePoint(initialBeam.p2, initialBeam.p1, angle);
                    angle += 15;
                } else {
                    initialBeam.p2 = rotatePoint(initialBeam.p2, initialBeam.p1, -angle*2);
                }
                moveRoght = !moveRoght;
                iterCounter++;
            }
        } 

        // if the initial beam does not intersect with any racetrack line, return
        if (first_intersection_index == -1) {
            RCLCPP_ERROR(this->get_logger(), "Initial beam does not intersect with any racetrack line");
            return;
        }

        // save the center of the first intersection as the first predicted point
        Point first_intersection_center(midpoint(racetrack_edges[first_intersection_index]->origP1, racetrack_edges[first_intersection_index]->origP2));
        predicted_points.push_back(first_intersection_center);
        // remove the first intersection from the racetrack_edges vector
        racetrack_edges.erase(racetrack_edges.begin() + first_intersection_index);

        // find the next 6 predicted points
        int last_intersection_index = first_intersection_index;
        while (predicted_points.size() < 12) {
            // shoot a 6m long beam from the last predicted point, which is perpendicular to the line defined by the two original points of the last racetrack line
            // unless the angle of the next cones is very extreme, the beam should intersect with the next racetrack line
            // TODO: if no intersection is found probably start searching in alternative angles left and right
            // Point last_predicted_point = predicted_points.back();
            // Point last_racetrack_line_center((racetrack_edges[last_intersection_index]->origP1.x + racetrack_edges[last_intersection_index]->origP2.x) / 2,
            //                                  (racetrack_edges[last_intersection_index]->origP1.y + racetrack_edges[last_intersection_index]->origP2.y) / 2);

            Point newPoint = perpendicularPoint(racetrack_edges[last_intersection_index]->origP1, racetrack_edges[last_intersection_index]->origP2, 6);
            
            // delete the last_intersection_index from the racetrack_edges vector, so we don't find the same itnersection twice (the points from where it's starting obviusly intersects our beam)
            // racetrack_edges.erase(racetrack_edges.begin() + last_intersection_index);
            racetrack_edges[last_intersection_index] = new vrn::VoronoiEdge(Point(1000, 1000), Point(1000, 1000), Point(1000, 1000), Point(1000, 1000)); // put this far away, as to not cause any more problems



            Line beam(predicted_points.back(), newPoint);

            // visualize the beam
            visualization_msgs::msg::MarkerArray marker_array_search_area;
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "laser_data_frame";
            marker.header.stamp = this->now();
            marker.ns = "search_area";
            marker.id = 0;
            marker.type = visualization_msgs::msg::Marker::LINE_LIST;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.1;
            marker.color.r = 1.0;
            marker.color.a = 1.0;
            geometry_msgs::msg::Point p1, p2;
            p1.x = beam.p1.x;
            p1.y = beam.p1.y;
            p2.x = beam.p2.x;
            p2.y = beam.p2.y;
            marker.points.push_back(p1);
            marker.points.push_back(p2);
            marker_array_search_area.markers.push_back(marker);
            search_area_pub->publish(marker_array_search_area);
            
            // find the first intersection of the beam with the racetrack line
            // and save the index of it in the racetrack_edges vector || this is first as in the first we find, might need to be refined to closest later TODO
            bool intersection_found = false;
            // for (int i = 0; i < racetrack_edges.size(); i++) {
            //     if (doIntersect(beam, Line(racetrack_edges[i]->origP1, racetrack_edges[i]->origP2))) {
            //         last_intersection_index = i;
            //         intersection_found = true;
            //         break;
            //     }
            // }

            int iterCounter = 0;
            int maxIter = 5;
            double angle = 22.5;
            bool moveRight = true;

            while (intersection_found == false && iterCounter < maxIter) {
                for (int i = 0; i < int(racetrack_edges.size()); i++) {
                    if (doIntersect(beam, Line(racetrack_edges[i]->origP1, racetrack_edges[i]->origP2))) {
                        last_intersection_index = i;
                        intersection_found = true;
                        break;
                    }
                }

                // if no intersection is found, rotate the beam slightly and try again
                if (intersection_found == false) {
                    if (moveRight) {
                        beam.p2 = rotatePoint(beam.p2, beam.p1, angle);
                    } else {
                        beam.p2 = rotatePoint(beam.p2, beam.p1, -angle*2);
                    }
                    moveRight = !moveRight;
                    iterCounter++;
                    angle += 15;
                }
            }

            // If the last index is the same as the current index, we have not found any intersection
            if (!intersection_found) {
                // RCLCPP_ERROR(this->get_logger(), "Beam does not intersect with any racetrack line after point %d", predicted_points.size());
                break;
            }
            

            // save the center of the first intersection as the next predicted point
            Point last_intersection_center(midpoint(racetrack_edges[last_intersection_index]->origP1, racetrack_edges[last_intersection_index]->origP2));
            predicted_points.push_back(last_intersection_center);
            
        }

        // rclcpp info the size
        RCLCPP_INFO(this->get_logger(), "predicted_points size: %d", int(predicted_points.size()));



        /****************************************************************
         *               Interesting bits ends here                     *
         *                                                              *
        *****************************************************************/













        /*
                    super messy markers, need to be cleaned up before release
                    ˇˇˇˇˇˇˇˇˇˇˇˇˇˇˇˇˇˇˇ
        */
        

        // visualization_msgs::msg::MarkerArray marker_array_pred;
        // int predindex = 0;  // Initialize the loop index
        // int num_points = predicted_points.size();  // Number of lines to be drawn

        // // put 0,0 as the first point of predicted point
        // predicted_points.insert(predicted_points.begin(), Point(0, 0));

        // for (int i = 1; i < num_points; i++) {
        //     visualization_msgs::msg::Marker marker;
        //     marker.header.frame_id = "laser_data_frame";
        //     marker.header.stamp = this->now();
        //     marker.ns = "predicted_point_" + std::to_string(predindex);  // Append the index to the namespace
        //     marker.id = predindex;  // Use the index as the marker ID
        //     marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        //     marker.action = visualization_msgs::msg::Marker::ADD;
        //     marker.pose.orientation.w = 1.0;
        //     marker.scale.x = 0.3;

        //     // // Calculate the color gradient from green to red
        //     // float ratio = static_cast<float>(i) / num_points;
        //     marker.color.r = 0.0;
        //     marker.color.g = 1.0;
        //     marker.color.b = 0.0;
        //     marker.color.a = 0.3;

        //     geometry_msgs::msg::Point p1, p2;
        //     p1.x = predicted_points[i-1].x;
        //     p1.y = predicted_points[i-1].y;
        //     p2.x = predicted_points[i].x;
        //     p2.y = predicted_points[i].y;
        //     marker.points.push_back(p1);
        //     marker.points.push_back(p2);

        //     marker_array_pred.markers.push_back(marker);
        //     ++predindex;  // Increment the loop index
        // }

        predicted_points.insert(predicted_points.begin(), Point(0, 0));
        visualization_msgs::msg::MarkerArray marker_array_pred = lineListMarker("laser_data_frame", "predicted_point_", 0.3, 0.0, 1.0, 0.0, 0.3, predicted_points);

        // to get rid of ghost markers
        max_markerarray = std::max(max_markerarray, int(marker_array.markers.size()));
        max_markerarray_pred = std::max(max_markerarray_pred, int(marker_array_pred.markers.size()));

        eliminateGhostMarkers(max_markerarray_pred, marker_array_pred.markers.size(), marker_array_pred, true);

        // Add markers for clusters that are not present in the current frame to avoid ghost markers
        for(int i = marker_array.markers.size(); i < max_markerarray; i++) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "laser_data_frame";
            marker.header.stamp = this->now();
            marker.ns = "voronoi_edges_" + std::to_string(i);
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::LINE_LIST;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.1;
            marker.color.b = 1.0;
            marker.color.a = 0.0;
            marker_array.markers.push_back(marker);
        }



        // publishing
        pred_point_pub->publish(marker_array_pred);
        voronoi_marker_pub->publish(marker_array);
        voronoi_orig_p_pub->publish(orig_ponts);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr voronoi_marker_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr voronoi_orig_p_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pred_point_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr search_area_pub;


    int max_markerarray = 0;
    int max_markerarray_pred = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VoronoiNode>());
    rclcpp::shutdown();
    return 0;
}
