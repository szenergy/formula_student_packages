import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
import joblib

class MarkerColorPublisher(Node):
    def __init__(self):
        super().__init__('marker_color_publisher')
        self.model = joblib.load('marker_classification_model.pkl') #load modell

        #sb topic
        self.marker_subscriber = self.create_subscription(
            MarkerArray,
            'pc_markers',
            self.marker_callback,
            10)

        #pl topic
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            'pc_markers_with_colors',
            10)

    def marker_callback(self, msg):
        #read data from topic
        points = []
        for marker in msg.markers:
            points.append((marker.pose.position.x, marker.pose.position.y, marker.pose.position.z))

        if not points:
            self.get_logger().warning("unable data")
            return

        X = [[point[0], point[1], point[2]] for point in points]#data structure
        predictions = self.model.predict(X) #add data to modell

        #publish combined data
        colored_markers = MarkerArray()
        for i, (marker, label) in enumerate(zip(msg.markers, predictions)):
            new_marker = Marker()
            new_marker.header = marker.header
            new_marker.ns = marker.ns
            new_marker.id = marker.id
            new_marker.type = marker.type
            new_marker.action = marker.action
            new_marker.pose = marker.pose
            new_marker.scale = marker.scale
            new_marker.lifetime = marker.lifetime
            new_marker.frame_locked = marker.frame_locked
            #colors
            if label == 'blue':
                new_marker.color.r = 0.0
                new_marker.color.g = 0.0
                new_marker.color.b = 1.0
                new_marker.color.a = 1.0
            elif label == 'yellow':
                new_marker.color.r = 1.0
                new_marker.color.g = 1.0
                new_marker.color.b = 0.0
                new_marker.color.a = 1.0
            #elif label == 'orange':      #need more training data!
            #    new_marker.color.r = 1.0
            #    new_marker.color.g = 1.0
            #    new_marker.color.b = 0.5
            #    new_marker.color.a = 1.0
            else:
                new_marker.color.r = 0.5
                new_marker.color.g = 0.5
                new_marker.color.b = 0.5
                new_marker.color.a = 1.0

            colored_markers.markers.append(new_marker)
        self.marker_publisher.publish(colored_markers)
        self.get_logger().info(f"{len(colored_markers.markers)}")


def main(args=None):
    rclpy.init(args=args)
    marker_publisher = MarkerColorPublisher()
    try:
        rclpy.spin(marker_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            marker_publisher.destroy_node()
        except Exception as e:
            marker_publisher.get_logger().error(f"error: {e}")
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
