import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
import numpy as np
import std_msgs.msg

class ConeRefinementNode(Node):

    def __init__(self):
        super().__init__('cone_refinement_node')

        self.subscription_pc = self.create_subscription(
            MarkerArray,
            'pc_markers',
            self.pc_callback,
            10)
        self.subscription_cones = self.create_subscription(
            MarkerArray,
            'deproj_cones',
            self.cones_callback,
            10)

        self.valid_blue_pub = self.create_publisher(MarkerArray, 'Valid_blue', 10)
        self.valid_yellow_pub = self.create_publisher(MarkerArray, 'Valid_yellow', 10)
        self.valid_orange_pub = self.create_publisher(MarkerArray, 'Valid_orange', 10)
        self.object_pub = self.create_publisher(Marker, 'distance_correspondences', 10)

        self.pc_positions = None
        self.cone_markers = None 

    def pc_callback(self, msg):
        self.pc_positions = np.array(
            [[marker.pose.position.x, marker.pose.position.y, marker.pose.position.z] for marker in msg.markers]
        )

    def cones_callback(self, msg):
        self.cone_markers = msg
        if self.pc_positions is not None:
            self.refine_cones()

    def refine_cones(self):
        if self.cone_markers is None:
            return

        blue_markers = MarkerArray()
        yellow_markers = MarkerArray()
        orange_markers = MarkerArray()

        for marker in self.cone_markers.markers:
            if marker.color.b == 1.0:  
                blue_markers.markers.append(marker)
            elif marker.color.r == 1.0 and marker.color.g == 1.0:  
                yellow_markers.markers.append(marker)
            elif marker.color.r == 1.0 and marker.color.g == 0.5:
                orange_markers.markers.append(marker)

        if blue_markers.markers:
            self.refine_and_publish_cones(blue_markers, 'blue', self.valid_blue_pub)
        if yellow_markers.markers:
            self.refine_and_publish_cones(yellow_markers, 'yellow', self.valid_yellow_pub)
        if orange_markers.markers:
            self.refine_and_publish_cones(orange_markers, 'orange', self.valid_orange_pub)

    def refine_and_publish_cones(self, cone_markers, cone_type, valid_cone_pub):
        cone_positions = np.array(
            [[marker.pose.position.x, marker.pose.position.y, marker.pose.position.z] for marker in cone_markers.markers]
        )

        if self.pc_positions is None or len(self.pc_positions) == 0:
            return
        if len(cone_positions) == 0:
            return

        if np.isnan(self.pc_positions).any() or np.isnan(cone_positions).any():
            return

        try:
            refined_positions = self.distance_based_refinement(self.pc_positions, cone_positions)
            self.publish_correspondence_objects(self.pc_positions, refined_positions, cone_type)
        except Exception as e:
            return

        refined_markers = MarkerArray()

        for i, marker in enumerate(cone_markers.markers):
            refined_marker = Marker()
            refined_marker = marker
            refined_marker.pose.position.x = refined_positions[i][0]
            refined_marker.pose.position.y = refined_positions[i][1]
            refined_marker.pose.position.z = refined_positions[i][2]
            refined_markers.markers.append(refined_marker)

        valid_cone_pub.publish(refined_markers)

    def distance_based_refinement(self, pc_positions, cone_positions):
        refined_positions = []

        for cone in cone_positions:
            distances = np.linalg.norm(pc_positions - cone, axis=1)
            nearest_idx = np.argmin(distances)
            refined_positions.append(pc_positions[nearest_idx])

        return np.array(refined_positions)

    def publish_correspondence_objects(self, source_positions, target_positions, cone_type):
        object_marker = Marker()
        object_marker.header.frame_id = "laser_sensor_frame"
        object_marker.type = Marker.LINE_LIST
        object_marker.action = Marker.ADD
        object_marker.scale.x = 0.02
        object_marker.scale.y = 0.02
        object_marker.scale.z = 3.0

        if cone_type == 'blue':
            object_marker.color.r = 0.0
            object_marker.color.g = 0.0
            object_marker.color.b = 1.0
        elif cone_type == 'yellow':
            object_marker.color.r = 1.0
            object_marker.color.g = 1.0
            object_marker.color.b = 0.0
        elif cone_type == 'orange':
            object_marker.color.r = 1.0
            object_marker.color.g = 0.5
            object_marker.color.b = 0.0

        object_marker.color.a = 1.0

        for source, target in zip(source_positions, target_positions):
            p1 = [source[0], source[1], source[2]]
            p2 = [target[0], target[1], target[2]]

            object_marker.points.append(self.create_point(p1))
            object_marker.points.append(self.create_point(p2))

        self.object_pub.publish(object_marker)

    def create_point(self, position):
        point = Point()
        point.x, point.y, point.z = position
        return point

def main(args=None):
    rclpy.init(args=args)
    cone_refinement_node = ConeRefinementNode()
    rclpy.spin(cone_refinement_node)

    cone_refinement_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()