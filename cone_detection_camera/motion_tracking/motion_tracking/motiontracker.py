import time
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
import numpy as np

class Track:
    def __init__(self, track_id, center, marker):
        self.id = track_id
        self.center = center
        self.marker = marker
        self.missed_frames = 0
        self.velocity = np.zeros(3)

class MarkerArrayHandler:
    def __init__(self, node, input_topic, output_topic, ns):
        self.node = node
        self.ns = ns
        self.tracks = []
        self.next_track_id = 0
        self.prev_marker_ids = set()
        self.latest_marker_array = None

        self.subscription = node.create_subscription(MarkerArray, input_topic, self.markerarray_callback, 10)
        self.publisher = node.create_publisher(MarkerArray, output_topic, 10)
        self.proximity_filter_enabled = node.get_parameter('proximity_filter_enabled').get_parameter_value().bool_value
        self.motion_tracking_enabled = node.get_parameter('motion_tracking_enabled').get_parameter_value().bool_value
        self.motion_matching_distance_threshold = node.get_parameter('motion_matching_distance_threshold').get_parameter_value().double_value
        self.max_missed_frames = node.get_parameter('max_missed_frames').get_parameter_value().integer_value

    def markerarray_callback(self, msg):
        start_time = time.time()
        self.node.get_logger().info(f'{self.ns}: MarkerArray üzenet érkezett.')
        candidate_markers = []
        for marker in msg.markers:
            marker.header.frame_id = 'laser_data_frame'
            marker.ns = self.ns
            center = np.array([marker.pose.position.x, marker.pose.position.y, marker.pose.position.z])
            candidate_markers.append((marker, center))
        if self.proximity_filter_enabled:
            excluded_indices = set()
            n_candidates = len(candidate_markers)
            for i in range(n_candidates):
                for j in range(i + 1, n_candidates):
                    center_i = candidate_markers[i][1]
                    center_j = candidate_markers[j][1]
                    if np.linalg.norm(center_i - center_j) < 1.0:
                        excluded_indices.add(i)
                        excluded_indices.add(j)
            candidate_markers = [candidate_markers[idx] for idx in range(n_candidates) if idx not in excluded_indices]
        if self.motion_tracking_enabled:
            self.update_tracks(candidate_markers)
            final_markers = [track.marker for track in self.tracks]
        else:
            final_markers = []
            self.tracks = []
            for marker, center in candidate_markers:
                marker.id = self.next_track_id
                new_track = Track(self.next_track_id, center, marker)
                self.next_track_id += 1
                self.tracks.append(new_track)
                final_markers.append(marker)
        new_marker_ids = {marker.id for marker in final_markers}
        deletion_markers = []
        for prev_id in self.prev_marker_ids:
            if prev_id not in new_marker_ids:
                delete_marker = Marker()
                delete_marker.header.frame_id = 'laser_data_frame'
                delete_marker.header.stamp = self.node.get_clock().now().to_msg()
                delete_marker.ns = self.ns
                delete_marker.id = prev_id
                delete_marker.action = Marker.DELETE
                deletion_markers.append(delete_marker)
        self.prev_marker_ids = new_marker_ids
        marker_array = MarkerArray()
        marker_array.markers = final_markers + deletion_markers
        self.publisher.publish(marker_array)
        self.latest_marker_array = marker_array
        end_time = time.time()
        elapsed_time = end_time - start_time
        self.node.get_logger().info(f'{self.ns}: MarkerArray publikálva. Feldolgozás {elapsed_time:.3f} másodpercet vett igénybe.')

    def update_tracks(self, candidate_markers):
        updated_track_ids = set()
        for candidate_marker, candidate_center in candidate_markers:
            best_track = None
            best_distance = float('inf')
            for track in self.tracks:
                dist = np.linalg.norm(candidate_center - track.center)
                if dist < best_distance:
                    best_distance = dist
                    best_track = track
            if best_track is not None and best_distance < self.motion_matching_distance_threshold:
                displacement = candidate_center - best_track.center
                best_track.velocity = displacement
                best_track.center = candidate_center
                best_track.marker = candidate_marker
                best_track.missed_frames = 0
                updated_track_ids.add(best_track.id)
                candidate_marker.id = best_track.id
            else:
                new_track = Track(self.next_track_id, candidate_center, candidate_marker)
                candidate_marker.id = self.next_track_id
                self.next_track_id += 1
                self.tracks.append(new_track)
                updated_track_ids.add(new_track.id)
        for track in self.tracks:
            if track.id not in updated_track_ids:
                track.missed_frames += 1
        self.tracks = [track for track in self.tracks if track.missed_frames < self.max_missed_frames]

class MultiMarkerArrayProcessor(Node):
    def __init__(self):
        super().__init__('multi_markerarray_processor')
        self.declare_parameter('proximity_filter_enabled', False)
        self.declare_parameter('motion_tracking_enabled', True)
        self.declare_parameter('motion_matching_distance_threshold', 1.0)
        self.declare_parameter('max_missed_frames', 3)
        self.declare_parameter('process_yellow_cones', True)
        self.declare_parameter('process_blue_cones', True)
        self.declare_parameter('process_green_cones', True)
        self.declare_parameter('publish_combined_topic', False)
        self.handlers = []
        if self.get_parameter('process_yellow_cones').get_parameter_value().bool_value:
            yellow_handler = MarkerArrayHandler(self, '/yellow_cones', 'processed_yellow_cones', 'yellow_cones')
            self.handlers.append(yellow_handler)
        if self.get_parameter('process_blue_cones').get_parameter_value().bool_value:
            blue_handler = MarkerArrayHandler(self, '/blue_cones', 'processed_blue_cones', 'blue_cones')
            self.handlers.append(blue_handler)
        if self.get_parameter('process_green_cones').get_parameter_value().bool_value:
            green_handler = MarkerArrayHandler(self, '/pc_markers', 'processed_green_cones', 'green_cones')
            self.handlers.append(green_handler)
        self.get_logger().info('MultiMarkerArrayProcessor elindult. Aktív feldolgozók: ' + ', '.join([h.ns for h in self.handlers]))
        self.publish_combined = self.get_parameter('publish_combined_topic').get_parameter_value().bool_value
        if self.publish_combined:
            self.combined_publisher = self.create_publisher(MarkerArray, 'processed_combined', 10)
            self.create_timer(0.1, self.publish_combined_callback)

    def publish_combined_callback(self):
        combined_markers = []
        for handler in self.handlers:
            if handler.latest_marker_array is not None:
                combined_markers.extend(handler.latest_marker_array.markers)
        if combined_markers:
            marker_array = MarkerArray()
            marker_array.markers = combined_markers
            self.combined_publisher.publish(marker_array)
            self.get_logger().info(f'Konzolidasztált MarkerArray publikálva {len(combined_markers)} marker-rel.')

def main(args=None):
    rclpy.init(args=args)
    node = MultiMarkerArrayProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node megszakítva.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
