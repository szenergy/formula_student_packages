#include <iostream>
#include <vector>
#include <boost/polygon/voronoi.hpp>
#include <functional>


// Namespace and type aliases for convenience
using boost::polygon::voronoi_builder;
using boost::polygon::voronoi_diagram;
using boost::polygon::x;
using boost::polygon::y;

/**
 * @brief Structure to represent a 2D point with double coordinates.
 */
struct Point {
    double x; ///< x-coordinate
    double y; ///< y-coordinate

    /**
     * @brief Default constructor to initialize a Point to (0, 0).
     */
    Point() : x(0), y(0) {}

    /**
     * @brief Constructor to initialize a Point with given coordinates.
     * @param x The x-coordinate.
     * @param y The y-coordinate.
     */
    Point(double x, double y) : x(x), y(y) {}

    /**
     * @brief Overload the == operator to compare two Point instances.
     * @param other The other Point to compare with.
     * @return True if the points have the same coordinates, false otherwise.
     */
    bool operator==(const Point& other) const {
        return x == other.x && y == other.y;
    }
};

/**
 * @brief Create a marker array for a list of points
 * @param frame_id The frame id of the marker
 * @param marker_ns The namespace of the marker (iteration will be added to this)
 * @param scale_x The scale of the marker
 * @param colour_r The red value of the marker
 * @param colour_g The green value of the marker
 * @param colour_b The blue value of the marker
 * @param colour_a The alpha value of the marker
 * @param points The list of points to be visualized
 * @param edit_points A function to edit the points (optional)
*/
visualization_msgs::msg::MarkerArray lineListMarker(std::string frame_id, std::string marker_ns, 
                    double scale_x, double colour_r, double colour_g, 
                    double colour_b, double colour_a, 
                    std::vector<Point> &points) {
    
    visualization_msgs::msg::MarkerArray marker_array;
    for (int i = 1; i < int(points.size()); i++){
        visualization_msgs::msg::Marker line_strip;
        line_strip.header.frame_id = frame_id;
        line_strip.header.stamp = rclcpp::Clock().now();
        line_strip.ns = marker_ns + std::to_string(i);
        line_strip.pose.orientation.w = 1.0;
        line_strip.id = i;
        line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_strip.action = visualization_msgs::msg::Marker::ADD;
        line_strip.scale.x = scale_x;
        line_strip.color.r = colour_r;
        line_strip.color.g = colour_g;
        line_strip.color.b = colour_b;
        line_strip.color.a = colour_a;
        geometry_msgs::msg::Point p1, p2;
        p1.x = points[i-1].x;
        p1.y = points[i-1].y;
        p2.x = points[i].x;
        p2.y = points[i].y;
        line_strip.points.clear();
        line_strip.points.push_back(p1);
        line_strip.points.push_back(p2);
        marker_array.markers.push_back(line_strip);
    }

    return marker_array;
}

/**
 * @brief Eliminate ghost markers by removing the excess markers
 * @param max_marker_size The maximum size of the marker array (reference to global variable)
 * @param marker_size The size of the marker array
 * @param marker_array The marker array to be edited
*/
void eliminateGhostMarkers(int &max_marker_size, int marker_size, 
                            visualization_msgs::msg::MarkerArray &marker_array, bool isLineList) {
    max_marker_size = std::max(max_marker_size, marker_size);

    for (int i = marker_size; i < max_marker_size; i++) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = marker_array.markers[0].header.frame_id;
        marker.header.stamp = rclcpp::Clock().now();
        std::string ns = marker_array.markers[0].ns;
        ns.pop_back();
        marker.ns = ns + std::to_string(i);
        marker.id = i;
        marker.type = isLineList ? visualization_msgs::msg::Marker::LINE_LIST : visualization_msgs::msg::Marker::POINTS;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker_array.markers.push_back(marker);
    }

}


// Boost polygon library specialization for custom Point structure
namespace boost {
namespace polygon {

// Specializing the geometry_concept for Point to use the point_concept
template <>
struct geometry_concept<Point> {
    typedef point_concept type;
};

// Specializing point_traits for Point to define how to access its coordinates
template <>
struct point_traits<Point> {
    typedef double coordinate_type; // Define the coordinate type as int

    // Function to get the coordinate of a Point based on orientation (horizontal or vertical)
    static inline coordinate_type get(
        const Point& point, orientation_2d orient) {
        return (orient == HORIZONTAL) ? point.x : point.y; // Return x or y coordinate
    }
};
}  // namespace polygon
}  // namespace boost

namespace vrn {

    // Class to represent a Voronoi edge. For each edge, we store the two endpoints, and the original points that were used to construct the edge
    class VoronoiEdge {
    public:
        // Constructor to initialize the Voronoi edge with its endpoints and the original points
        VoronoiEdge(const Point& p1, const Point& p2, const Point& orig1, const Point& orig2)
            : edgeP1(p1), edgeP2(p2), origP1(orig1), origP2(orig2) {}

        Point edgeP1; // First endpoint of the edge
        Point edgeP2; // Second endpoint of the edge
        Point origP1; // First original point used to construct the edge
        Point origP2; // Second original point used to construct the edge

    };

    // Class to represent and handle Voronoi diagram operations
    class VoronoiDiagram {
    public:
        // Constructor that takes a vector of Points and constructs the Voronoi diagram
        VoronoiDiagram(const std::vector<Point>& points)
            : points_(points) {
            construct_voronoi(points_.begin(), points_.end(), &vd_);
        }

        // Function to get all the edges of the Voronoi diagram as pairs of Points
        std::vector<std::pair<Point, Point>> getEdges() const {
            std::vector<std::pair<Point, Point>> edges;
            for (auto it = vd_.edges().begin(); it != vd_.edges().end(); ++it) {
                if (it->is_primary()) { // Only consider primary edges
                    const auto* v0 = it->vertex0();
                    const auto* v1 = it->vertex1();
                    if (v0 && v1) { // Ensure both vertices of the edge exist
                        edges.emplace_back(Point(v0->x(), v0->y()), Point(v1->x(), v1->y()));
                    }
                }
            }
            return edges;
        }

        // Function to get all the edges of the Voronoi diagram as VoronoiEdge objects
        std::vector<VoronoiEdge> getVoronoiEdges() const {
            std::vector<VoronoiEdge> voronoi_edges;
            for (auto it = vd_.edges().begin(); it != vd_.edges().end(); ++it) {
                if (it->is_primary()) { // Only consider primary edges
                    const auto* v0 = it->vertex0();
                    const auto* v1 = it->vertex1();
                    if (v0 && v1) { // Ensure both vertices of the edge exist
                        const auto* cell0 = it->cell();
                        const auto* cell1 = it->twin()->cell();
                        const auto& orig0 = points_[cell0->source_index()];
                        const auto& orig1 = points_[cell1->source_index()];
                        voronoi_edges.emplace_back(Point(v0->x(), v0->y()), Point(v1->x(), v1->y()), orig0, orig1);
                    }
                }
            }
            return voronoi_edges;
        }



        // Function to print all the edges of the Voronoi diagram to the standard output
        void printVoronoiEdges() const {
            auto edges = getEdges();
            for (const auto& edge : edges) {
                std::cout << "((" << edge.first.x << ", " << edge.first.y << "), ("
                        << edge.second.x << ", " << edge.second.y << "))," << std::endl;
            }
        }

        // Function to get the underlying Voronoi diagram object
        const voronoi_diagram<double>& getVoronoiDiagram() const {
            return vd_;
        }

    private:
        std::vector<Point> points_; // Vector of points used to construct the Voronoi diagram
        voronoi_diagram<double> vd_; // The Voronoi diagram object
    }; // class VoronoiDiagram



} /// namespace vrn