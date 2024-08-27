import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
import math

#Azért tervez rá ennyire a jobb oldalra, mert csak azt nézi a kód egyenlőre. 

#Annyi a lényege hogy minden esetben menjen, ha nem látszik a sáv széle akkor is.
#Jelenleg a sárga(jobb oldali) és a gmap-ből visszaadott lokál szűrés re van beállítva.

#Kettő ilyen párhuzamosan futó résszel megoldható a kéken és a sárgán, hogy minden esetben legyen tervezett jó útvonal.

#Zöld pontok: A zöld pontok a két egymás 
#utáni sárga bója által meghatározott vonal középpontjától 
#indulva, 1,5 méterrel balra helyezkednek el. 
#Ez azt jelenti, hogy a zöld pontok nem közvetlenül a sárga 
#bójákra merőlegesen helyezkednek el, hanem a két bója által 
#meghatározott szakasz középpontjára.

#Piros pontok: A piros pontok a két egymás 
#utáni sárga bója által meghatározott vonalra merőleges irányban, 
#1,5 méterrel balra helyezkednek el az első bójától (p1) számítva.

class ConeSubscriber(Node):
    def __init__(self):
        super().__init__('cone_subscriber')
        self.yellow_cones = []
        self.yellow_subscriber = self.create_subscription(
            MarkerArray,
            'local_fov_markers',
            self.yellow_callback,
            10
        )
        self.points_publisher = self.create_publisher(
            MarkerArray,
            'planned_points',
            10
        )
        self.lines_publisher = self.create_publisher(
            MarkerArray,
            'planning_lines',
            10
        )
        self.midpoint_publisher = self.create_publisher(
            MarkerArray,
            'midpoint_points',
            10
        )
        self.marker_id_counter = 0
        self.line_id_counter = 0
        self.midpoint_id_counter = 0

    def yellow_callback(self, msg):
        self.yellow_cones = [marker for marker in msg.markers if self.is_yellow(marker.color)]
        self.publish_points_and_lines()
    def is_yellow(self, color):
        return color.r == 1.0 and color.g == 1.0 and color.b == 0.0 and color.a == 1.0

    def publish_points_and_lines(self):
        #törlés
        delete_marker_array = MarkerArray()
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        delete_marker_array.markers.append(delete_marker)
        self.points_publisher.publish(delete_marker_array)
        self.lines_publisher.publish(delete_marker_array)
        self.midpoint_publisher.publish(delete_marker_array)

        if not self.yellow_cones:
            return

        point_markers, line_markers, midpoint_markers = self.create_points_and_lines()

        points_marker_array = MarkerArray()
        points_marker_array.markers = point_markers

        lines_marker_array = MarkerArray()
        lines_marker_array.markers = line_markers

        midpoint_marker_array = MarkerArray()
        midpoint_marker_array.markers = midpoint_markers

        self.points_publisher.publish(points_marker_array)
        self.lines_publisher.publish(lines_marker_array)
        self.midpoint_publisher.publish(midpoint_marker_array)

    def create_points_and_lines(self):
        point_markers = []
        line_markers = []
        midpoint_markers = []

        for i in range(len(self.yellow_cones) - 1):
            p1 = self.yellow_cones[i].pose.position
            p2 = self.yellow_cones[i + 1].pose.position
            #skip:
            if self.are_points_too_close(p1, p2):
                continue
            if self.distance_between_points(p1, p2) > 6.0:
                continue

            left_point = self.calculate_left_point(p1, p2, 1.5)

            point_marker = self.create_marker(left_point, self.marker_id_counter, 'planned_points', color=(1.0, 0.0, 0.0))
            self.marker_id_counter += 1
            point_markers.append(point_marker)

            line_marker = Marker()
            line_marker.header.frame_id = self.yellow_cones[i].header.frame_id
            line_marker.header.stamp = self.get_clock().now().to_msg()
            line_marker.ns = 'planning_lines'
            line_marker.id = self.line_id_counter
            self.line_id_counter += 1
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.scale.x = 0.05  
            line_marker.color.r = 0.0
            line_marker.color.g = 0.0
            line_marker.color.b = 1.0
            line_marker.color.a = 1.0 
            line_marker.points.append(p1)
            line_marker.points.append(p2)

            line_marker.lifetime = rclpy.duration.Duration(seconds=0.0).to_msg()
            line_markers.append(line_marker)

            midpoint = self.calculate_midpoint(p1, p2)
            perpendicular_point = self.calculate_left_point(p1, p2, 1.5, midpoint)

            midpoint_marker = self.create_marker(perpendicular_point, self.midpoint_id_counter, 'midpoint_points', color=(0.0, 1.0, 0.0))
            self.midpoint_id_counter += 1
            midpoint_markers.append(midpoint_marker)

        return point_markers, line_markers, midpoint_markers

    def calculate_midpoint(self, p1, p2):
        midpoint = Point()
        midpoint.x = (p1.x + p2.x) / 2
        midpoint.y = (p1.y + p2.y) / 2
        midpoint.z = (p1.z + p2.z) / 2
        return midpoint

    def calculate_left_point(self, p1, p2, distance, origin=None):
        if origin is None:
            origin = p1

        dx = p2.x - p1.x
        dy = p2.y - p1.y


        length = math.sqrt(dx**2 + dy**2)
        if length == 0:
            return origin 

        dx /= length
        dy /= length

        left_dx = -dy
        left_dy = dx

        new_point = Point()
        new_point.x = origin.x + left_dx * distance
        new_point.y = origin.y + left_dy * distance
        new_point.z = origin.z

        return new_point

    def are_points_too_close(self, p1, p2, tolerance=1e-6):
        distance = math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)
        return distance < tolerance

    def distance_between_points(self, p1, p2):
        return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

    def create_marker(self, position, marker_id, namespace, color):
        marker = Marker()
        marker.header.frame_id = 'laser_sensor_frame'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = position
        marker.scale.x = 0.2 
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0 
        marker.lifetime = rclpy.duration.Duration(seconds=0.0).to_msg()
        return marker

def main(args=None):
    rclpy.init(args=args)
    node = ConeSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
