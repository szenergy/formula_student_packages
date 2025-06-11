import rclpy
from rclpy.node import Node
import numpy as np
import math
import time
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float64, Float64MultiArray
from fsd_path_planning import MissionTypes, PathPlanner

class Fsd(Node):
    def __init__(self):
        super().__init__("ft_fsd")
        self.declare_parameter('use_slam', False)
        use_slam = self.get_parameter('use_slam').get_parameter_value().bool_value
        self.declare_parameter('use_blue_markers', True)
        self.declare_parameter('blue_marker_topic', '/detected_cones')
        use_blue = self.get_parameter('use_blue_markers').get_parameter_value().bool_value
        blue_topic = self.get_parameter('blue_marker_topic').get_parameter_value().string_value
        self.declare_parameter('use_yellow_markers', False)
        self.declare_parameter('yellow_marker_topic', '/Valid_yellow')
        use_yellow = self.get_parameter('use_yellow_markers').get_parameter_value().bool_value
        yellow_topic = self.get_parameter('yellow_marker_topic').get_parameter_value().string_value
        self.declare_parameter('path_planning.max_planning_distance', 30.0)
        self.declare_parameter('lookahead_distance', 5.0)
        self.lookahead_distance = self.get_parameter('lookahead_distance').get_parameter_value().double_value
        self.declare_parameter('mission_type', "trackdrive")
        mission_type_str = self.get_parameter('mission_type').get_parameter_value().string_value.lower()
        if mission_type_str == "skidpad":
            self.mission_type = MissionTypes.skidpad
            self.get_logger().info("Mission type: skidpad")
        elif mission_type_str == "acceleration":
            self.mission_type = MissionTypes.acceleration
            self.get_logger().info("Mission type: acceleration")
        else:
            self.mission_type = MissionTypes.trackdrive
            self.get_logger().info("Mission type: trackdrive")
        if use_slam:
            self.pose_sub = self.create_subscription(PoseStamped, '/pose', self.pose_cb, 1)
            self.frame_id = "map"
        else:
            self.frame_id = "laser_data_frame"
        if use_blue:
            self.blue_cone_sub = self.create_subscription(MarkerArray, blue_topic, self.blue_marker_cb, 1)
            self.get_logger().info(f"Subscribing to blue markers on: {blue_topic}")
        else:
            self.get_logger().info("Blue marker subscription disabled.")
        if use_yellow:
            self.yellow_cone_sub = self.create_subscription(MarkerArray, yellow_topic, self.yellow_marker_cb, 1)
            self.get_logger().info(f"Subscribing to yellow markers on: {yellow_topic}")
        else:
            self.get_logger().info("Yellow marker subscription disabled.")
        self.raceline_publisher = self.create_publisher(Path, "/raceline", 1)
        self.trajectory_publisher = self.create_publisher(Path, "/trajectory", 1)
        self.steering_angle_pub = self.create_publisher(Float64, "/steering_angle", 1)
        self.lane_boundaries_pub = self.create_publisher(MarkerArray, "/lane_boundaries", 1)
        self.control_pub = self.create_publisher(Float64MultiArray, "/control_commands", 1)
        self.steering_angle_subscriber = self.create_subscription(Float64, "/steering_angle", self.steering_angle_callback, 10)
        self.car_position = np.array([0.0, 0.0])
        self.angle = 0.0
        self.latest_blue_markers = None
        self.latest_yellow_markers = None
        self.latest_trajectory = None
        self.last_pose_time = None
        self.last_car_position = None
        self.last_speed = 0.0
        self.acceleration = 0.0
        self.start_time = time.time()
        self.steering_times = []
        self.steering_angles = []
        self.speed_times = []
        self.speed_values = []
        self.brake_values = []
        plt.ion()
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, sharex=True)
        self.line_steering, = self.ax1.plot([], [])
        self.ax1.axhline(0, color='black', lw=1)
        self.line_speed, = self.ax2.plot([], [])
        self.ax2.axhline(0, color='black', lw=1)
        self.ax2.set_ylim(0, 0)
        self.ax2_brake = self.ax2.twinx()
        self.line_brake, = self.ax2_brake.plot([], [], color='red')
        self.ax2_brake.set_ylim(0, 0)
        self.fig.show()
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.kalman_x = 0.0
        self.kalman_P = 1.0
        self.kalman_Q = 0.01
        self.kalman_R = 0.1

    def pose_cb(self, pose: PoseStamped):
        current_time = self.get_clock().now().nanoseconds * 1e-9
        current_position = np.array([pose.pose.position.x, pose.pose.position.y])
        self.car_position = current_position
        if self.last_pose_time is not None:
            dt = current_time - self.last_pose_time
            if dt > 0:
                displacement = np.linalg.norm(current_position - self.last_car_position)
                speed = displacement / dt
                self.acceleration = (speed - self.last_speed) / dt
                self.last_speed = speed
        self.last_pose_time = current_time
        self.last_car_position = current_position

    def blue_marker_cb(self, marker_array: MarkerArray):
        self.latest_blue_markers = marker_array
        self.process_markers()

    def yellow_marker_cb(self, marker_array: MarkerArray):
        self.latest_yellow_markers = marker_array
        self.process_markers()

    def process_markers(self):
        blue_cones = self.marker_array_to_positions(self.latest_blue_markers)
        yellow_cones = self.marker_array_to_positions(self.latest_yellow_markers)
        if blue_cones.shape[0] == 0 and yellow_cones.shape[0] == 0:
            self.get_logger().warn("Received empty cone markers, skipping")
            return
        cone_array = [blue_cones, yellow_cones, np.empty((0, 2)), np.empty((0, 2)), np.empty((0, 2))]
        try:
            planner = PathPlanner(self.mission_type)
            out = planner.calculate_path_in_global_frame(cone_array, self.car_position, self.angle, return_intermediate_results=True)
        except Exception as e:
            self.get_logger().error(f"Error in ft_fsd: {e}")
            return
        trajectory = out[0]
        blue_sorted = out[1]
        yellow_sorted = out[2]
        alpha = 0.4
        if self.latest_trajectory is not None and trajectory.shape == self.latest_trajectory.shape:
            self.latest_trajectory = alpha * trajectory + (1 - alpha) * self.latest_trajectory
        else:
            self.latest_trajectory = trajectory
        poses = []
        for (x, y) in self.latest_trajectory[:, 1:3]:
            ps = PoseStamped()
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.header.frame_id = self.frame_id
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = 0.0
            ps.pose.orientation.x = 0.0
            ps.pose.orientation.y = 0.0
            ps.pose.orientation.z = 0.0
            ps.pose.orientation.w = 1.0
            poses.append(ps)
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.frame_id
        path_msg.poses = poses
        self.raceline_publisher.publish(path_msg)
        self.trajectory_publisher.publish(path_msg)
        self.get_logger().info("Path planner alive and publishing trajectory", throttle_duration_sec=10.0)
        marker_array_msg = MarkerArray()
        blue_marker = Marker()
        blue_marker.header.frame_id = self.frame_id
        blue_marker.header.stamp = self.get_clock().now().to_msg()
        blue_marker.ns = "lane_boundaries"
        blue_marker.id = 0
        blue_marker.type = Marker.LINE_STRIP
        blue_marker.action = Marker.ADD
        blue_marker.scale.x = 0.1
        blue_marker.color.r = 0.0
        blue_marker.color.g = 0.0
        blue_marker.color.b = 1.0
        blue_marker.color.a = 1.0
        for p in blue_sorted:
            pt = Point()
            pt.x = float(p[0])
            pt.y = float(p[1])
            pt.z = 0.0
            blue_marker.points.append(pt)
        marker_array_msg.markers.append(blue_marker)
        yellow_marker = Marker()
        yellow_marker.header.frame_id = self.frame_id
        yellow_marker.header.stamp = self.get_clock().now().to_msg()
        yellow_marker.ns = "lane_boundaries"
        yellow_marker.id = 1
        yellow_marker.type = Marker.LINE_STRIP
        yellow_marker.action = Marker.ADD
        yellow_marker.scale.x = 0.1
        yellow_marker.color.r = 1.0
        yellow_marker.color.g = 1.0
        yellow_marker.color.b = 0.0
        yellow_marker.color.a = 1.0
        for p in yellow_sorted:
            pt = Point()
            pt.x = float(p[0])
            pt.y = float(p[1])
            pt.z = 0.0
            yellow_marker.points.append(pt)
        marker_array_msg.markers.append(yellow_marker)
        self.lane_boundaries_pub.publish(marker_array_msg)

    def timer_callback(self):
        if self.latest_trajectory is not None:
            steering_angle = self.compute_steering_angle(self.latest_trajectory)
            filtered_angle = self.kalman_filter_update(steering_angle)
            steering_angle_deg = np.degrees(filtered_angle)
            angle_msg = Float64()
            angle_msg.data = steering_angle_deg
            self.steering_angle_pub.publish(angle_msg)
            control_msg = Float64MultiArray()
            control_msg.data = [steering_angle_deg, self.last_speed]
            self.control_pub.publish(control_msg)
            current_time = time.time() - self.start_time
            self.steering_times.append(current_time)
            self.steering_angles.append(steering_angle_deg)
            self.speed_times.append(current_time)
            self.speed_values.append(self.last_speed)
            brake_force = max(0, -self.acceleration)
            self.brake_values.append(brake_force)
            min_time = max(0, current_time - 10)
            self.ax1.set_xlim(min_time, current_time)
            self.ax2.set_xlim(min_time, current_time)
            self.ax2_brake.set_xlim(min_time, current_time)
            self.line_steering.set_data(self.steering_times, self.steering_angles)
            self.line_speed.set_data(self.speed_times, self.speed_values)
            self.line_brake.set_data(self.speed_times, self.brake_values)
            self.ax1.relim()
            self.ax1.autoscale_view(True, True, True)
            self.ax2.set_ylim(0, 0)
            #self.ax2_brake.set_ylim(0, 10)
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            self.get_logger().info(f"Current speed: {self.last_speed:.2f} m/s", throttle_duration_sec=1.0)
        else:
            self.get_logger().warn("No trajectory available yet.")

    def compute_steering_angle(self, trajectory: np.ndarray) -> float:
        if trajectory.shape[0] == 0:
            return 0.0
        target_point = None
        for point in trajectory:
            x = point[1]
            y = point[2]
            distance = np.linalg.norm(np.array([x, y]) - self.car_position)
            if distance >= self.lookahead_distance:
                target_point = np.array([x, y])
                break
        if target_point is None:
            target_point = np.array([trajectory[-1, 1], trajectory[-1, 2]])
        dx = target_point[0] - self.car_position[0]
        dy = target_point[1] - self.car_position[1]
        desired_angle = np.arctan2(dy, dx)
        relative_angle = desired_angle - self.angle
        relative_angle = np.arctan2(np.sin(relative_angle), np.cos(relative_angle))
        return relative_angle

    def kalman_filter_update(self, measurement: float) -> float:
        x_pred = self.kalman_x
        P_pred = self.kalman_P + self.kalman_Q
        K = P_pred / (P_pred + self.kalman_R)
        self.kalman_x = x_pred + K * (measurement - x_pred)
        self.kalman_P = (1 - K) * P_pred
        return self.kalman_x

    def marker_array_to_positions(self, marker_array):
        if marker_array is None:
            return np.empty((0, 2))
        positions = []
        for marker in marker_array.markers:
            positions.append([-marker.pose.position.x, marker.pose.position.y])
        if len(positions) == 0:
            return np.empty((0, 2))
        return np.array(positions)
    
    def steering_angle_callback(self, msg: Float64):
        self.get_logger().info(f"steering angle: {msg.data:.3f} d")

def main(args=None):
    rclpy.init(args=args)
    fsd = Fsd()
    rclpy.spin(fsd)
    rclpy.shutdown()

if __name__ == "__main__":
    main()