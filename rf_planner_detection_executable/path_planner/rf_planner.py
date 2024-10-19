import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
import joblib
import numpy as np
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

class RealTimeTester(Node):
    def __init__(self):
        super().__init__('real_time_tester')
        self.marker_subscriber = self.create_subscription(
            MarkerArray,
            'pc_markers',
            self.marker_callback,
            10)

        #creat spline
        self.spline_marker = Marker()
        self.spline_marker.header.frame_id = "laser_sensor_frame"
        self.spline_marker.header.stamp = self.get_clock().now().to_msg()
        self.spline_marker.ns = "spline"
        self.spline_marker.id = 0
        self.spline_marker.type = Marker.LINE_STRIP
        self.spline_marker.action = Marker.ADD
        self.spline_marker.scale.x = 0.1 
        self.spline_marker.color.r = 0.0
        self.spline_marker.color.g = 1.0
        self.spline_marker.color.b = 0.0
        self.spline_marker.color.a = 1.0

        self.spline_publisher = self.create_publisher(Marker, 'spline_marker', 10)
        self.model = joblib.load('test/legjobblidar.pkl')

    def marker_callback(self, msg):
        points = []
        for marker in msg.markers:
            points.append((marker.pose.position.x, marker.pose.position.y, marker.pose.position.z))
        
        if points:
            avg_x = sum(p[0] for p in points) / len(points)
            avg_y = sum(p[1] for p in points) / len(points)
            prediction = (self.model.predict(np.array([[avg_x, avg_y]]))) * 100
            self.get_logger().info(f"Kanyarszög: {prediction[0]} fok")

            self.update_spline(prediction[0])

    def update_spline(self, turn_angle):
        start_x, start_y = 0.0, 0.0
        car_length = 2.5  
        max_distance = 5.0

        turn_angle_rad = np.radians(turn_angle)

        if turn_angle_rad != 0:
            radius = car_length / np.tan(turn_angle_rad)
        else:
            radius = float('inf')
        points = []
        num_points = 50  

        for i in range(num_points + 1):
            distance = (i / num_points) * max_distance
            if radius == float('inf'):
                x = start_x + distance
                y = start_y
            else:
                angle = distance / radius
                x = start_x + radius * np.sin(angle)
                y = start_y - radius * (1 - np.cos(angle))
            points.append((x, y, 0.0))

        self.spline_marker.points = []
        for (x, y, z) in points:
            point = Point()
            point.x = float(x)  
            point.y = float(y)  
            point.z = float(z)
            self.spline_marker.points.append(point)
        self.spline_marker.header.stamp = self.get_clock().now().to_msg()
        self.spline_publisher.publish(self.spline_marker)

def main(args=None):
    rclpy.init(args=args)
    tester = RealTimeTester()
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        tester.get_logger().info("done...")
    finally:
        tester.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
