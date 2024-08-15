import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile
from visualization_msgs.msg import Marker, MarkerArray
import copy

class GlobalMappingNode(Node):

    def __init__(self):
        super().__init__('mapping_node')

        # Gv
        self.alpha = 0.9
        self.max_hits = 1000
        self.min_hits = -8 #-5
        self.R_max_cov = 1.5 #close
        self.R_min_cov = 0.1
        self.freq_output = 10
        self.delta_cov = self.R_max_cov * (self.max_hits - 1) / self.max_hits**2
        self.cone_db = np.empty((0, 7))  # Initialize with 0 rows and 7 columns
        self.dist_FOV_max = 10.0 #10
        self.dist_FOV_min = 1.7 #1.7
        self.dist_FOV_max2 = 10.0
        self.dist_FOV_min2 = 0.5
        self.angle_FOV = 45 #90
        self.angle_FOV2 = 90
        self.last_pose = PoseStamped()
        self.freq_update_db = 10
        self.add_hit = 3
        self.sub_hit = 1
        self.transformed_cones = []
        self.legit_cone_hits = 10000
        self.freq_clean_db = 1
        self.id_cone = 0
        self.default_x = 640113.5
        self.default_y = 5193671.0
        self.distance = 0
        #sub:
        self.create_subscription(PoseStamped, '/current_pose_fake_orientation', self.callback_pose, QoSProfile(depth=10))
        self.create_subscription(MarkerArray, '/yellow_cones', self.callback_yellow_cones, QoSProfile(depth=10))
        self.create_subscription(MarkerArray, '/blue_cones', self.callback_blue_cones, QoSProfile(depth=10))
        self.create_subscription(MarkerArray, '/orange_cones', self.callback_orange_cones, QoSProfile(depth=10))
        #pub:
        self.publisher_rviz = self.create_publisher(MarkerArray, '/global_map_markers', QoSProfile(depth=10))
        self.publisher_local_fov = self.create_publisher(MarkerArray, '/local_fov_markers', QoSProfile(depth=10))
        
        self.timer_update_db = self.create_timer(1.0/self.freq_update_db, self.update_cone_db)

        self.T = np.zeros((1, 2))
        self.theta = 0.0
        self.R_plus = np.eye(2)
        self.R_minus = np.eye(2)
        self.pose = np.zeros(3)

        self.reactive_cones_yellow = np.empty((0, 3))  # Initialize with 0 rows and 3 columns
        self.reactive_cones_blue = np.empty((0, 3))
        self.reactive_cones_orange = np.empty((0, 3))

    def callback_pose(self, msg):
        self.last_pose = msg
        self.T = np.array((msg.pose.position.x - self.default_x, msg.pose.position.y - self.default_y)).reshape(1, 2)
        orientation_q = msg.pose.orientation
        self.theta = self.get_yaw_from_quaternion(orientation_q)

        self.R_plus = np.array([[np.cos(self.theta), -np.sin(self.theta)], [np.sin(self.theta), np.cos(self.theta)]]).reshape(2, 2)
        self.R_minus = np.array([[np.cos(-self.theta), -np.sin(-self.theta)], [np.sin(-self.theta), np.cos(-self.theta)]]).reshape(2, 2)

        self.pose = np.array([msg.pose.position.x - self.default_x, msg.pose.position.y -self.default_y, self.theta], dtype=np.float32)

    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return np.arctan2(siny_cosp, cosy_cosp)
        
    def callback_yellow_cones(self, msg):
        self.reactive_cones_yellow = self.process_cones(msg, 1)
    def callback_blue_cones(self, msg):
        self.reactive_cones_blue = self.process_cones(msg, 2)
    def callback_orange_cones(self, msg):
        self.reactive_cones_orange = self.process_cones(msg, 3)
    def process_cones(self, msg, color):
        cones = []
        for marker in msg.markers:
            if marker.pose.position.x == 0 or marker.pose.position.y == 0:
                continue  # Skip this cone if any coordinate is zero
            cones.append([marker.pose.position.x, marker.pose.position.y, color])
        
        cones = np.array(cones)
        return cones

    def update_cone_db(self):
        reactive_cones_list = [arr for arr in [self.reactive_cones_yellow, self.reactive_cones_blue, self.reactive_cones_orange] if arr.size > 0]
        if reactive_cones_list:
            self.reactive_cones = np.vstack(reactive_cones_list)
        else:
            self.reactive_cones = np.empty((0, 3))

        try:
            reactive_cones = self.reactive_cones
            R_plus = self.R_plus
            R_minus = self.R_minus
            T = self.T
        except:
            return

        reactive_in_fov_index = []

        for i in range(reactive_cones.shape[0]):
            c = reactive_cones[i, :]

            angle = (np.arctan2(c[1], c[0]) * 180 / np.pi)
            dist = np.sqrt(c[1]**2 + c[0]**2)

            if dist <= self.dist_FOV_max and angle <= self.angle_FOV and angle >= -self.angle_FOV:
                reactive_in_fov_index.append(i)

        if len(reactive_in_fov_index) == 0:
            return

        reactive_in_fov_local = copy.deepcopy(reactive_cones[reactive_in_fov_index, :])

        reactive_in_fov_local = np.hstack((reactive_in_fov_local, np.zeros((reactive_in_fov_local.shape[0], 4))))
        reactive_in_fov_local[:, 3] = self.R_max_cov

        reactive_in_fov_map = copy.deepcopy(reactive_in_fov_local)

        if reactive_in_fov_map.shape[0] > 1:
            reactive_in_fov_map[:, 0:2] = np.matmul(reactive_in_fov_map[:, 0:2], R_minus)
            reactive_in_fov_map[:, 0:2] = reactive_in_fov_map[:, 0:2] + T
        elif reactive_in_fov_map.shape[0] == 1:
            reactive_in_fov_map = reactive_in_fov_map.reshape(1, 7)
            reactive_in_fov_map[0, 0:2] = np.matmul(reactive_in_fov_map[0, 0:2], R_minus)
            reactive_in_fov_map[0, 0:2] = reactive_in_fov_map[0, 0:2] + T

        for i in range(reactive_in_fov_map.shape[0]):
            reactive = reactive_in_fov_map[i]
            orientation_data = [self.pose[0], self.pose[1]]
            if np.allclose(reactive[0:2], orientation_data, atol=0.5):
                self.cone_db = np.delete(self.cone_db, np.argwhere(np.allclose(self.cone_db[:, 0:2], orientation_data, atol=0.01)), axis=0)

        if self.cone_db.shape[0] == 0:
            self.cone_db = copy.deepcopy(reactive_in_fov_map)
            self.output_4_rviz(self.cone_db.astype(np.float32))
            return
        else:
            self.get_logger().info(f"Aviable cones count: {str(self.cone_db.shape[0])}")

        reactive_in_db = []
        db_in_reactive = []
        for i in range(reactive_in_fov_map.shape[0]):
            reactive = reactive_in_fov_map[i]
            dist_db = np.linalg.norm(self.cone_db[:, 0:2] - reactive[0:2], axis=1)
            if np.amin(dist_db) <= self.R_max_cov:
                reactive_in_db.append(i)
                db_in_reactive.append(np.argmin(dist_db))
        reactive_in_fov_map_new = np.delete(reactive_in_fov_map, reactive_in_db, axis=0)

        dataICP = copy.deepcopy(self.pose).reshape(1, 3)
        for i in range(len(reactive_in_db)):
            i_react = reactive_in_db[i]
            i_db = db_in_reactive[i]
            react = np.array([reactive_in_fov_map[i_react, 0], reactive_in_fov_map[i_react, 1], 0]).reshape(1, 3)
            db = np.array([self.cone_db[i_db, 0], self.cone_db[i_db, 1], 0]).reshape(1, 3)

            dataICP = np.vstack((dataICP, db, react))

        self.cone_db[db_in_reactive, 4] = self.cone_db[db_in_reactive, 4] + self.add_hit
        self.cone_db[np.argwhere(self.cone_db[:, 4] > self.max_hits), 4] = self.max_hits
        self.cone_db[np.argwhere(self.cone_db[:, 4] < self.min_hits), 4] = self.min_hits
        self.cone_db[db_in_reactive, 0:2] = (1 - self.alpha) * reactive_in_fov_map[reactive_in_db, 0:2] + (self.alpha) * self.cone_db[db_in_reactive, 0:2]
        self.cone_db[:, 3] = self.R_max_cov - self.delta_cov * self.cone_db[:, 4]
        self.cone_db[np.argwhere(self.cone_db[:, 3] > self.R_max_cov), 3] = self.R_max_cov
        self.cone_db[np.argwhere(self.cone_db[:, 3] < self.R_min_cov), 3] = self.R_min_cov

        self.cone_db = np.vstack((self.cone_db, reactive_in_fov_map_new))

        no_id_index = np.argwhere(self.cone_db[:, 5] == 0)
        for id_cone in no_id_index:
            self.cone_db[id_cone, 5] = self.id_cone
            self.id_cone = self.id_cone + 1

        db_local = copy.deepcopy(self.cone_db)

        if db_local.shape[0] > 1:
            db_local[:, 0:2] = db_local[:, 0:2] - T
            db_local[:, 0:2] = np.matmul(db_local[:, 0:2], R_plus)
        elif db_local.shape[0] == 1:
            db_local = db_local.reshape(1, 7)
            db_local[0, 0:2] = db_local[0, 0:2] - T
            db_local[0, 0:2] = np.matmul(db_local[0, 0:2], R_plus)

        db_local_distance = np.linalg.norm(db_local[:, 0:2], axis=1)
        db_local_in_distance_index = np.argwhere(np.logical_and(db_local_distance <= self.dist_FOV_max, db_local_distance >= self.dist_FOV_min))

        db_local_in_angle_index = []

        for index_cone in db_local_in_distance_index:
            x = db_local[index_cone, 0]
            y = db_local[index_cone, 1]
            angle = np.arctan2(y, x) * 180 / np.pi
            if angle <= self.angle_FOV and angle >= -self.angle_FOV:
                db_local_in_angle_index.append(index_cone)

        self.cone_db[db_local_in_angle_index, 5] = 1
        
        for index in db_local_in_angle_index:
            if index not in db_in_reactive[:]:
                self.cone_db[index, 4] = self.cone_db[index, 4] - self.sub_hit

        self.cone_db = np.delete(self.cone_db, np.argwhere(self.cone_db[:, 4] <= self.min_hits), axis=0)
        self.output_4_rviz(self.cone_db.astype(np.float32))
        self.cone_db[:, 5] = 0

        self.update_local_fov()

    def update_local_fov(self):
        try:
            db_cones = copy.deepcopy(self.cone_db)
            T = self.T
            R_plus = self.R_plus
        except:
            return

        if db_cones.shape[0] > 1:
            db_cones[:, 0:2] = db_cones[:, 0:2] - T
            db_cones[:, 0:2] = np.matmul(db_cones[:, 0:2], R_plus)
        elif db_cones.shape[0] == 1:
            db_cones = db_cones.reshape(1, 7)
            db_cones[0, 0:2] = db_cones[0, 0:2] - T
            db_cones[0, 0:2] = np.matmul(db_cones[0, 0:2], R_plus)

        db_local_distance = np.linalg.norm(db_cones[:, 0:2], axis=1)
        db_local_in_distance_index = np.argwhere(np.logical_and(db_local_distance <= self.dist_FOV_max2, db_local_distance >= self.dist_FOV_min2))

        db_local_in_angle_index = []

        for index_cone in db_local_in_distance_index:
            x = db_cones[index_cone, 0]
            y = db_cones[index_cone, 1]
            angle = np.arctan2(y, x) * 180 / np.pi
            if angle <= self.angle_FOV2 and angle >= -self.angle_FOV2:
                db_local_in_angle_index.append(index_cone[0])

        db_local_in_angle_index = np.array(db_local_in_angle_index).astype(int).flatten()  # Ensure indices are integers
        local_fov_cones = db_cones[db_local_in_angle_index]
        self.output_local_fov(local_fov_cones.astype(np.float32))

    def output_4_rviz(self, rviz_cones):
        marker_array = MarkerArray()
        if rviz_cones.size == 0:
            self.get_logger().info("No cones to visualize")
            return
        
        for i, cone in enumerate(rviz_cones):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "cones"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(cone[0])
            marker.pose.position.y = float(cone[1])
            marker.pose.position.z = float(0)
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.color.a = 1.0

            cone_id = int(cone[2])
            if cone_id == 1:  #Y
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            elif cone_id == 2:  #B
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
            elif cone_id == 3:  #O
                marker.color.r = 1.0
                marker.color.g = 0.5
                marker.color.b = 0.0
            else:
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 1.0  

            marker_array.markers.append(marker)
        self.publisher_rviz.publish(marker_array)

    def output_local_fov(self, local_fov_cones):
        marker_array = MarkerArray()
        if local_fov_cones.size == 0:
            self.get_logger().info("No local FOV cones to visualize")
            return

        for i, cone in enumerate(local_fov_cones):
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "local_fov_cones"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(cone[0])
            marker.pose.position.y = float(cone[1])
            marker.pose.position.z = float(0)
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.color.a = 1.0

            cone_id = int(cone[2])
            if cone_id == 1:  #Y
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            elif cone_id == 2:  #B
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
            elif cone_id == 3:  #O
                marker.color.r = 1.0
                marker.color.g = 0.5
                marker.color.b = 0.0
            else:
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 1.0  

            marker_array.markers.append(marker)
        self.publisher_local_fov.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = GlobalMappingNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
