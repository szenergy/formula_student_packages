import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from sklearn.cluster import DBSCAN
import struct
from concurrent.futures import ThreadPoolExecutor
#rom simplefuse import ConeRefinementNode #object_fusion class

class PointCloudProcessor(Node): #pc clastering node

    def __init__(self):
        super().__init__('pointcloud_processor')
        self.point_subscriber = self.create_subscription(PointCloud2, 'nonground', self.pointcloud_callback, 10)
        self.marker_publisher = self.create_publisher(MarkerArray, 'cp_markers', 10)
        self.declare_parameter('cluster_distance', 0.4)
        self.declare_parameter('batch_size', 10000)
        self.max_distance = 20.0
        self.min_height = -1.0
        self.max_height = -0.5
        self.voxel_size = 0.1

    def pointcloud_callback(self, msg):
        start_time = time.time()
        self.get_logger().info('Received a PointCloud2 message.')
        points = self.extract_and_filter_points_from_pointcloud2(msg)
        
        if len(points) == 0:
            self.get_logger().info('No points remaining after filtering.')
            return
        self.get_logger().info(f'Number of points after filtering: {len(points)}')

        points = self.mirror_points_across_x_axis(points)

        cluster_distance = self.get_parameter('cluster_distance').get_parameter_value().double_value
        batch_size = self.get_parameter('batch_size').get_parameter_value().integer_value

        marker_array = MarkerArray()
        clusters = self.process_point_cloud_in_batches(points, batch_size, cluster_distance)
        
        for i, cluster in enumerate(clusters):
            if len(cluster) > 0:
                center = np.mean(cluster, axis=0)
                marker = self.create_cylinder_marker(center, i)
                marker_array.markers.append(marker)

        self.marker_publisher.publish(marker_array)
        end_time = time.time()
        elapsed_time = end_time - start_time
        self.get_logger().info(f'Published MarkerArray. Processing took {elapsed_time:.3f} seconds.')

    def extract_and_filter_points_from_pointcloud2(self, cloud_msg):
        points = []
        point_step = cloud_msg.point_step
        x_offset = cloud_msg.fields[0].offset
        y_offset = cloud_msg.fields[1].offset
        z_offset = cloud_msg.fields[2].offset

        for i in range(0, len(cloud_msg.data), point_step):
            x = struct.unpack_from('f', cloud_msg.data, i + x_offset)[0]
            y = struct.unpack_from('f', cloud_msg.data, i + y_offset)[0]
            z = struct.unpack_from('f', cloud_msg.data, i + z_offset)[0]

            distance_from_origin = np.sqrt(x**2 + y**2 + z**2)
        
            if distance_from_origin < 0.5:
                continue
            distance = np.sqrt(x**2 + y**2 + z**2)
            if distance <= self.max_distance and self.min_height <= z <= self.max_height:
                points.append([x, y, z])

        points = np.array(points)
        if len(points) > 0:
            points = self.voxel_grid_filter(points, self.voxel_size)
        return points

    def mirror_points_across_x_axis(self, points):
        mirrored_points = points.copy()
        mirrored_points[:, 1] = -mirrored_points[:, 1]
        mirrored_points[:, 0] = -mirrored_points[:, 0]
        return mirrored_points

    def voxel_grid_filter(self, points, voxel_size):
        voxel_indices = np.floor(points / voxel_size).astype(np.int32)
        _, unique_indices = np.unique(voxel_indices, axis=0, return_index=True)
        print("voxel")
        print(type(points[unique_indices]))
        print(type(points[unique_indices][0]))
        return points[unique_indices]

    def process_point_cloud_in_batches(self, points, batch_size, cluster_distance):
        num_batches = (len(points) + batch_size - 1) // batch_size
        clusters = []

        with ThreadPoolExecutor() as executor:
            futures = [
                executor.submit(self.cluster_points, points[i*batch_size:(i+1)*batch_size], cluster_distance)
                for i in range(num_batches)
            ]
            for future in futures:
                
                clusters.extend(future.result())
        return clusters

    #dbscan cluter point (need to do something)
    def cluster_points(self, points, cluster_distance):
        try:
            clusters = points.tolist()
            clustering = DBSCAN(eps=cluster_distance, min_samples=2).fit(points)
            labels = clustering.labels_
            clusters = [points[labels == i] for i in range(max(labels) + 1) if i != -1]
        except Exception as e:
            self.get_logger().error(f'Error clustering points: {e}')
            clusters = []
        return clusters

    def create_cylinder_marker(self, position, marker_id):
        marker = Marker()
        marker.header.frame_id = 'laser_sensor_frame'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'cylinder'
        marker.id = marker_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = float(position[0])
        marker.pose.position.y = float(position[1])
        marker.pose.position.z = float(position[2])
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 0.5
        marker.color.g = 0.0
        marker.color.b = 0.5
        marker.color.a = 1.0
        return marker

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudProcessor()
    #node2 = ConeRefinementNode() 
    try:
       # rclpy.spin(node, node2)
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted.')
    finally:
        node.destroy_node()
        #node2.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()