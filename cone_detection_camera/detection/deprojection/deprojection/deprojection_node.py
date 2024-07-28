import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from scipy.spatial.transform import Rotation
import time
#main
class Deproject:
    def __init__(self, roll_deg=0, pitch_deg=-2.0, yaw_deg=0, height=1.602, 
                 K_vec=[335.20611572265625, 0.0, 331.85467529296875, 0.0, 335.20611572265625, 183.79928588867188, 0.0, 0.0, 1.0],
                 R_vec=[np.pi / 2, - np.pi / 2, 0]):
        
        self.h = height
        self.R_vec = R_vec
        self.K_vec = K_vec

        self.pitch = np.deg2rad(pitch_deg)
        self.roll = np.deg2rad(roll_deg)
        self.yaw = np.deg2rad(yaw_deg)

        self.R_cr = None
        self.R_rc = None
        self.t_rc = None
        self.R_gr = None
        self.t_gr = None
        self.K = None
        self.K_inv = None

        self.calculate_transforms(R_vec)
        self.calculate_intrinsics(K_vec)

    def calculate_transforms(self, R):
        cy, sy = np.cos(self.yaw), np.sin(self.yaw)
        cp, sp = np.cos(self.pitch), np.sin(self.pitch)
        cr, sr = np.cos(self.roll), np.sin(self.roll)

        self.R_cr = np.array(
            [[cr*cy+sp*sr*sy, cr*sp*sy-cy*sr, -cp*sy],
             [cp*sr,           cp*cr,         sp],
             [cr*sy-cy*sp*sr, -cr*cy*sp-sr*sy, cp*cy]]
        )

        self.R_rc = self.R_cr.T
        self.t_rc = np.array([0.06, -self.h, 0])

        self.R_gr = np.asarray(
            Rotation.from_euler('yxz', np.array(self.R_vec)).as_matrix()
        )
        self.t_gr = None

    def calculate_intrinsics(self, K):
        self.K = np.array(K).reshape(3, 3)
        self.K_inv = np.linalg.inv(self.K)

    def deproject_pixel(self, u, v):
        uv_h = np.array([u, v, 1])
        n_r = np.array([0, 1, 0])

        K_inv_matmul_uv_h = self.K_inv @ uv_h
        n_c = self.R_cr @ n_r
        denominator = np.dot(n_c, K_inv_matmul_uv_h)
        
        return self.h * K_inv_matmul_uv_h / denominator

    def transform_cam_to_road(self, vector):
        return self.R_rc @ vector + self.t_rc

    def transform_road_to_ground_link(self, vector):
        return self.R_gr @ vector

    def run(self, u, v):
        vector = self.deproject_pixel(u, v)
        vector = self.transform_cam_to_road(vector)
        vector = self.transform_road_to_ground_link(vector)
        return vector

class DeprojectionNode(Node):
    def __init__(self):
        super().__init__('deprojection_node')
        self.deprojector = Deproject()
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'cone_coordinates',
            self.listener_callback,
            10
        )
        self.publisher_yellow = self.create_publisher(MarkerArray, 'yellow_cones', 10)
        self.publisher_orange = self.create_publisher(MarkerArray, 'orange_cones', 10)
        self.publisher_blue = self.create_publisher(MarkerArray, 'blue_cones', 10)
        self.pub_proj_coords = self.create_publisher(Float32MultiArray, 'pub_proj_coords', 10)
        
        self.markers_yellow = {}
        self.markers_orange = {}
        self.markers_blue = {}
        self.active_marker_ids_yellow = set()
        self.active_marker_ids_orange = set()
        self.active_marker_ids_blue = set()
        self.timer = self.create_timer(0.1, self.timer_callback)

    def listener_callback(self, msg):
        if len(msg.data) % 5 != 0:
            return

        current_time = time.time()
        pub_msg = Float32MultiArray()

        for i in range(0, len(msg.data), 5):
            try:
                cone_id = int(msg.data[i])
                u = msg.data[i + 1]
                v = msg.data[i + 2]
                if v == 0.0:
                    continue
            except IndexError:
                continue

            point = self.deprojector.run(u, v)
            marker_id = f"{u}_{v}"
            marker_info = {
                'position': point,
                'timestamp': current_time
            }
            
            pub_msg.data.append(cone_id)
            pub_msg.data.append(point[0])
            pub_msg.data.append(point[1])

            if cone_id == 1:
                self.markers_yellow[marker_id] = marker_info
            elif cone_id == 3:
                self.markers_orange[marker_id] = marker_info
            elif cone_id == 2:
                self.markers_blue[marker_id] = marker_info
        #print(pub_msg)
        self.pub_proj_coords.publish(pub_msg)

    def timer_callback(self):
        self.publish_markers(self.markers_yellow, self.publisher_yellow, self.active_marker_ids_yellow)
        self.publish_markers(self.markers_orange, self.publisher_orange, self.active_marker_ids_orange)
        self.publish_markers(self.markers_blue, self.publisher_blue, self.active_marker_ids_blue)

    def publish_markers(self, markers, publisher, active_marker_ids):
        current_time = time.time()
        marker_array = MarkerArray()
        new_markers = {}
        used_marker_ids = set()

        for marker_id, marker_info in list(markers.items()):
            if current_time - marker_info['timestamp'] > 0.1:
                del markers[marker_id]
            else:
                keep_marker = True
                for new_marker_id, new_marker_info in new_markers.items():
                    distance = np.linalg.norm(np.array(marker_info['position']) - np.array(new_marker_info['position']))
                    if distance < 0.4:
                        keep_marker = False
                        break
                if keep_marker:
                    new_markers[marker_id] = marker_info

        markers.update(new_markers)

        for idx, (marker_id, marker_info) in enumerate(markers.items()):
            marker = Marker()
            marker.header.frame_id = "laser_sensor_frame"
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.color.a = 1.0
            if publisher == self.publisher_yellow:
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.3
            
            if publisher == self.publisher_blue:
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
            
            if publisher == self.publisher_orange:
                marker.color.r = 1.0
                marker.color.g = 0.5
                marker.color.b = 0.1
            
            marker.pose.position.x = marker_info['position'][0]
            marker.pose.position.y = marker_info['position'][1]
            marker.pose.position.z = marker_info['position'][2]
            marker.id = idx
            marker_array.markers.append(marker)
            used_marker_ids.add(idx)

        stale_marker_ids = active_marker_ids - used_marker_ids
        for stale_id in stale_marker_ids:
            marker = Marker()
            marker.action = Marker.DELETE
            marker.id = stale_id
            marker_array.markers.append(marker)

        active_marker_ids.clear()
        active_marker_ids.update(used_marker_ids)

        publisher.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = DeprojectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
