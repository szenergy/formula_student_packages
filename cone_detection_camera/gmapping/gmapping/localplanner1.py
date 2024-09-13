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

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
import math
import numpy as np
from rclpy.duration import Duration

class ConeSubscriber(Node):
    def __init__(self):
        super().__init__('cone_subscriber')
        self.yellow_cones = []
        self.blue_cones = []

        # Paraméterek hozzáadása a topicok kiválasztásához
        self.declare_parameter('use_local_fov', True)
        self.declare_parameter('use_source_topic', False)

        use_local_fov = self.get_parameter('use_local_fov').get_parameter_value().bool_value
        use_source_topic = self.get_parameter('use_source_topic').get_parameter_value().bool_value

        # A forrás topic alapján állítjuk be a subscription-öket
        if use_local_fov:
            self.yellow_subscriber = self.create_subscription(
                MarkerArray,
                'local_fov_markers',
                self.yellow_callback,
                10
            )
            self.blue_subscriber = self.create_subscription(
                MarkerArray,
                'local_fov_markers',
                self.blue_callback,
                10
            )

        if use_source_topic:
            self.blue_subscriber = self.create_subscription(
                MarkerArray,
                'blue_cones',
                self.blue_callback,
                10
            )
            self.yellow_subscriber = self.create_subscription(
                MarkerArray,
                'yellow_cones',
                self.yellow_callback,
                10
            )
        
        self.yellow_points_publisher = self.create_publisher(
            MarkerArray,
            'planned_points_yellow',
            10
        )
        self.yellow_lines_publisher = self.create_publisher(
            MarkerArray,
            'planning_lines_yellow',
            10
        )
        self.yellow_midpoint_publisher = self.create_publisher(
            MarkerArray,
            'midpoint_points_yellow',
            10
        )

        self.blue_points_publisher = self.create_publisher(
            MarkerArray,
            'planned_points_blue',
            10
        )
        self.blue_lines_publisher = self.create_publisher(
            MarkerArray,
            'planning_lines_blue',
            10
        )
        self.blue_midpoint_publisher = self.create_publisher(
            MarkerArray,
            'midpoint_points_blue',
            10
        )

        self.marker_id_counter_yellow = 0
        self.line_id_counter_yellow = 0
        self.midpoint_id_counter_yellow = 0
        
        self.marker_id_counter_blue = 0
        self.line_id_counter_blue = 0
        self.midpoint_id_counter_blue = 0

    def yellow_callback(self, msg):
        self.yellow_cones = [marker for marker in msg.markers if self.is_yellow(marker.color)]
        self.publish_points_and_lines('yellow')

    def blue_callback(self, msg):
        self.blue_cones = [marker for marker in msg.markers if self.is_blue(marker.color)]
        self.publish_points_and_lines('blue')

    def is_yellow(self, color):
        return color.r == 1.0 and color.g == 1.0 and color.b == 0.0 and color.a == 1.0

    def is_blue(self, color):
        return color.r == 0.0 and color.g == 0.0 and color.b == 1.0 and color.a == 1.0

    def publish_points_and_lines(self, color):
        if color == 'yellow':
            cones = self.yellow_cones
            marker_id_counter = self.marker_id_counter_yellow
            line_id_counter = self.line_id_counter_yellow
            midpoint_id_counter = self.midpoint_id_counter_yellow
            points_publisher = self.yellow_points_publisher
            lines_publisher = self.yellow_lines_publisher
            midpoint_publisher = self.yellow_midpoint_publisher
        elif color == 'blue':
            cones = self.blue_cones
            marker_id_counter = self.marker_id_counter_blue
            line_id_counter = self.line_id_counter_blue
            midpoint_id_counter = self.midpoint_id_counter_blue
            points_publisher = self.blue_points_publisher
            lines_publisher = self.blue_lines_publisher
            midpoint_publisher = self.blue_midpoint_publisher

        delete_marker_array = MarkerArray()
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        delete_marker_array.markers.append(delete_marker)
        points_publisher.publish(delete_marker_array)
        lines_publisher.publish(delete_marker_array)
        midpoint_publisher.publish(delete_marker_array)

        if not cones:
            return

        point_markers, line_markers, midpoint_markers, generated_points = self.create_points_and_lines(cones, marker_id_counter, line_id_counter, midpoint_id_counter, color)

        points_marker_array = MarkerArray()
        points_marker_array.markers = point_markers

        lines_marker_array = MarkerArray()
        lines_marker_array.markers = line_markers

        midpoint_marker_array = MarkerArray()
        midpoint_marker_array.markers = midpoint_markers

        points_publisher.publish(points_marker_array)
        lines_publisher.publish(lines_marker_array)
        midpoint_publisher.publish(midpoint_marker_array)

        self.fit_and_publish_polynomial(generated_points, lines_publisher, line_id_counter)

    def create_points_and_lines(self, cones, marker_id_counter, line_id_counter, midpoint_id_counter, color):
        point_markers = []
        line_markers = []
        midpoint_markers = []
        generated_points = []

        other_cones = self.blue_cones if color == 'yellow' else self.yellow_cones

        for i in range(len(cones) - 1):
            p1 = cones[i].pose.position
            p2 = cones[i + 1].pose.position

            if self.are_points_too_close(p1, p2):
                continue

            if self.distance_between_points(p1, p2) > 6.0:
                continue

            if color == 'yellow':
                left_point = self.calculate_left_point(p1, p2, 1.5)
            elif color == 'blue':
                left_point = self.calculate_right_point(p1, p2, 1.5)

            if self.is_too_close_to_other_cones(left_point, other_cones):
                continue

            generated_points.append(left_point)

            point_marker = self.create_marker(left_point, marker_id_counter, 'planned_points', color=(1.0, 0.0, 0.0))
            marker_id_counter += 1
            point_markers.append(point_marker)

            line_marker = Marker()
            line_marker.header.frame_id = cones[i].header.frame_id
            line_marker.header.stamp = self.get_clock().now().to_msg()
            line_marker.ns = 'planning_lines'
            line_marker.id = line_id_counter
            line_id_counter += 1
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.scale.x = 0.05
            line_marker.color.r = 0.0
            line_marker.color.g = 0.0
            line_marker.color.b = 1.0
            line_marker.color.a = 1.0

            line_marker.points.append(p1)
            line_marker.points.append(p2)

            line_marker.lifetime = Duration(seconds=0.0).to_msg()

            line_markers.append(line_marker)

            midpoint = self.calculate_midpoint(p1, p2)

            if color == 'yellow':
                perpendicular_point = self.calculate_left_point(p1, p2, 1.5, midpoint)
            elif color == 'blue':
                perpendicular_point = self.calculate_right_point(p1, p2, 1.5, midpoint)

            if not self.is_too_close_to_other_cones(perpendicular_point, other_cones):
                midpoint_marker = self.create_marker(perpendicular_point, midpoint_id_counter, 'midpoint_points', color=(0.0, 1.0, 0.0))
                midpoint_id_counter += 1
                midpoint_markers.append(midpoint_marker)

        return point_markers, line_markers, midpoint_markers, generated_points

    def fit_and_publish_polynomial(self, points, publisher, line_id_counter):
        if len(points) < 2: #2
            return

        x_coords = np.array([point.x for point in points])
        y_coords = np.array([point.y for point in points])

        poly_coeffs = np.polyfit(x_coords, y_coords, 2)  # Második fokú polinom illesztése
        poly_func = np.poly1d(poly_coeffs)

        poly_x = np.linspace(min(x_coords), max(x_coords), 100)
        poly_y = poly_func(poly_x)

        line_marker = Marker()
        line_marker.header.frame_id = 'laser_sensor_frame'
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.ns = 'fitted_polynomial'
        line_marker.id = line_id_counter
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.1
        line_marker.color.r = 1.0
        line_marker.color.g = 0.0
        line_marker.color.b = 0.0
        line_marker.color.a = 1.0

        for x, y in zip(poly_x, poly_y):
            point = Point()
            point.x = x
            point.y = y
            point.z = 0.0
            line_marker.points.append(point)

        # Csomagoljuk a line_marker-t egy MarkerArray objektumba
        marker_array = MarkerArray()
        marker_array.markers.append(line_marker)

        publisher.publish(marker_array)

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

    def calculate_right_point(self, p1, p2, distance, origin=None):
        if origin is None:
            origin = p1

        dx = p2.x - p1.x
        dy = p2.y - p1.y

        length = math.sqrt(dx**2 + dy**2)

        if length == 0:
            return origin

        dx /= length
        dy /= length

        right_dx = dy  # Invert the direction for right
        right_dy = -dx

        new_point = Point()
        new_point.x = origin.x + right_dx * distance
        new_point.y = origin.y + right_dy * distance
        new_point.z = origin.z

        return new_point

    def are_points_too_close(self, p1, p2, tolerance=1):
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
        marker.color.a = 1.0  # Fully opaque
        marker.lifetime = Duration(seconds=0.0).to_msg()
        return marker

    def is_too_close_to_other_cones(self, point, other_cones, threshold=1.2):
        for cone in other_cones:
            cone_position = cone.pose.position
            distance = self.distance_between_points(point, cone_position)
            if distance < threshold:
                return True
        return False

def main(args=None):
    rclpy.init(args=args)
    node = ConeSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
