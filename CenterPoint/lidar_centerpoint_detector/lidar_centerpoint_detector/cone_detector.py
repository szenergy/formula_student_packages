from timeit import default_timer

import numpy as np
import torch
from pyquaternion import Quaternion
#from det3d import __version__, torchie
from det3d.models import build_detector
from det3d.torchie import Config
from det3d.core.input.voxel_generator import VoxelGenerator

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import ros2_numpy
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from autoware_auto_perception_msgs.msg import PredictedObject, PredictedObjects, Shape, ObjectClassification
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
#from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray


class NeuralNetwork:
    def __init__(self, config_path, model_path):
        self.points = None
        self.config_path = config_path
        self.model_path = model_path
        self.device = None
        self.net = None
        self.voxel_generator = None
        self.inputs = None
        
    def initialize(self):
        self.read_config()
        
    def read_config(self):
        config_path = self.config_path
        cfg = Config.fromfile(self.config_path)
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.net = build_detector(cfg.model, train_cfg=None, test_cfg=cfg.test_cfg)
        self.net.load_state_dict(torch.load(self.model_path)["state_dict"])
        self.net = self.net.to(self.device).eval()

        self.range = cfg.voxel_generator.range
        self.voxel_size = cfg.voxel_generator.voxel_size
        self.max_points_in_voxel = cfg.voxel_generator.max_points_in_voxel
        self.max_voxel_num = cfg.voxel_generator.max_voxel_num
        self.voxel_generator = VoxelGenerator(
            voxel_size=self.voxel_size,
            point_cloud_range=self.range,
            max_num_points=self.max_points_in_voxel,
            max_voxels=self.max_voxel_num,
        )
    
    def get_annotations_indices(self, types, thresh, label_preds, scores):
        indexs = []
        annotation_indices = []
        for i in range(label_preds.shape[0]):
            if label_preds[i] == types:
                indexs.append(i)
        for index in indexs:
            if scores[index] >= thresh:
                annotation_indices.append(index)
        return annotation_indices
    
    def remove_low_score_nu(self, image_anno, thresh):
        img_filtered_annotations = {}
        label_preds_ = image_anno["label_preds"].detach().cpu().numpy()
        scores_ = image_anno["scores"].detach().cpu().numpy()
        
        # car_indices =                  self.get_annotations_indices(0, 0.4, label_preds_, scores_)
        # truck_indices =                self.get_annotations_indices(1, 0.4, label_preds_, scores_)
        # construction_vehicle_indices = self.get_annotations_indices(2, 0.4, label_preds_, scores_)
        # bus_indices =                  self.get_annotations_indices(3, 0.3, label_preds_, scores_)
        # trailer_indices =              self.get_annotations_indices(4, 0.4, label_preds_, scores_)
        # barrier_indices =              self.get_annotations_indices(5, 0.4, label_preds_, scores_)
        # motorcycle_indices =           self.get_annotations_indices(6, 0.15, label_preds_, scores_)
        # bicycle_indices =              self.get_annotations_indices(7, 0.15, label_preds_, scores_)
        # pedestrain_indices =           self.get_annotations_indices(8, 0.1, label_preds_, scores_)
        traffic_cone_indices =         self.get_annotations_indices(9, 0.4, label_preds_, scores_)
        
        for key in image_anno.keys():
            if key == 'metadata':
                continue
            # img_filtered_annotations[key] = (
            #     image_anno[key][car_indices +
            #                     pedestrain_indices + 
            #                     bicycle_indices +
            #                     bus_indices +
            #                     construction_vehicle_indices +
            #                     traffic_cone_indices +
            #                     trailer_indices +
            #                     barrier_indices +
            #                     truck_indices
            #                     ])
            img_filtered_annotations[key] = (
                image_anno[key][
                                traffic_cone_indices
                                ])

        return img_filtered_annotations

    def run(self, points):
        t_t = default_timer()
        print(f"input points shape: {points.shape}")
        num_features = 5        
        self.points = points.reshape([-1, num_features])
        self.points[:, 4] = 0 # timestamp value 
        
        voxels, coords, num_points = self.voxel_generator.generate(self.points)
        num_voxels = np.array([voxels.shape[0]], dtype=np.int64)
        grid_size = self.voxel_generator.grid_size
        coords = np.pad(coords, ((0, 0), (1, 0)), mode='constant', constant_values = 0)
        
        voxels = torch.tensor(voxels, dtype=torch.float32, device=self.device)
        coords = torch.tensor(coords, dtype=torch.int32, device=self.device)
        num_points = torch.tensor(num_points, dtype=torch.int32, device=self.device)
        num_voxels = torch.tensor(num_voxels, dtype=torch.int32, device=self.device)
        
        self.inputs = dict(
            voxels = voxels,
            num_points = num_points,
            num_voxels = num_voxels,
            coordinates = coords,
            shape = [grid_size]
        )
        torch.cuda.synchronize()
        t = default_timer()

        with torch.no_grad():
            outputs = self.net(self.inputs, return_loss=False)[0]
    
        # print(f"output: {outputs}")
        
        torch.cuda.synchronize()
        print("  network predict time cost:", default_timer() - t)

        outputs = self.remove_low_score_nu(outputs, 0.45)

        boxes_lidar = outputs["box3d_lidar"].detach().cpu().numpy()
        print("  predict boxes:", boxes_lidar.shape)

        scores = outputs["scores"].detach().cpu().numpy()
        types = outputs["label_preds"].detach().cpu().numpy()

        boxes_lidar[:, -1] = -boxes_lidar[:, -1] - np.pi / 2

        print(f"  total cost time: {default_timer() - t_t}")

        return scores, boxes_lidar, types


class ConeDetectorRosNode(Node):
    def __init__(self) -> None:
        super().__init__("cone_detector_node")
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_ALL,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # declare rosparams
        if not self.has_parameter("/cone_detector/lidar_input_topic"):
            self.declare_parameters(
                namespace="",
                parameters=[
                    ("/cone_detector/lidar_input_topic", '/nonground'),
                    #("/cone_detector/lidar_input_topic", '/points'),
                ],
            )

        # create publishers/subscribers
        # timer_period = 0.033  # [s]
        # self.timer = self.create_timer(timer_period, self.callback)
        #self.cmd_pub = self.create_publisher(BoundingBoxArray, "/pp_boxes", 1)
        config_path = '/home/dobayt/ros2_ws/src/CenterPoint/configs/nusc/pp/nusc_centerpoint_pp_02voxel_two_pfn_10sweep.py'
        model_path = '/home/dobayt/ros2_ws/src/CenterPoint/lidar_centerpoint_detector/lidar_centerpoint_detector/models/latest.pth'

        self.proc_1 = NeuralNetwork(config_path, model_path)
        self.proc_1.initialize()

        self.pcl_sub = self.create_subscription(
            PointCloud2, self.get_parameter("/cone_detector/lidar_input_topic").value, self.lidar_callback, 10
        )

        self.pub_arr_bbox = self.create_publisher(MarkerArray, "/detected_cones", 1)
        
        # example log
        self.get_logger().info(f"Inputs for the model: ")
        self.get_logger().info(f"Inputs for the controller: ")

    # HELPER FUNCTIONS
    def yaw2quaternion(self, yaw: float) -> Quaternion:
        return Quaternion(axis=[0,0,1], radians=yaw)
    
    def get_xyz_points(self, cloud_array, remove_nans=True, dtype=float):
        '''
        '''
        # if remove_nans:
        mask = np.isfinite(cloud_array['x']) & np.isfinite(cloud_array['y']) & np.isfinite(cloud_array['z'])
        cloud_array = cloud_array[mask]

        points = np.zeros(cloud_array.shape + (5,), dtype=dtype)
        points[...,0] = cloud_array['x']
        points[...,1] = cloud_array['y']
        points[...,2] = cloud_array['z']
        points[...,3] = cloud_array['intensity']
        return points

    def xyz_array_to_pointcloud2(self, points_sum, stamp=None, frame_id=None):
        '''
        Create a sensor_msgs.PointCloud2 from an array of points.
        '''
        msg = PointCloud2()
        if stamp:
            msg.header.stamp = stamp
        if frame_id:
            msg.header.frame_id = frame_id
        msg.height = 1
        msg.width = points_sum.shape[0]
        msg.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)
            # PointField('i', 12, PointField.FLOAT32, 1)
            ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = points_sum.shape[0]
        msg.is_dense = int(np.isfinite(points_sum).all())
        msg.data = np.asarray(points_sum, np.float32).tostring()
        return msg
    
    # CALLBACKS
    def lidar_callback(self, msg):
        t_t = default_timer()
        arr_bbox = MarkerArray()
        #arr_bbox = PredictedObjects()

        msg_cloud = ros2_numpy.point_cloud2.pointcloud2_to_array(msg)
        np_p = self.get_xyz_points(msg_cloud, True)
        scores, dt_box_lidar, types = self.proc_1.run(np_p)

        if scores.size != 0:
            for i in range(scores.size):
                # ---------- MARKER FORMAT ----------------
                marker = Marker()
                marker.header.frame_id = msg.header.frame_id
                marker.header.stamp = msg.header.stamp
                marker.ns = "bounding_boxes"
                marker.id = i
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.pose.position.x = float(dt_box_lidar[i][0])
                marker.pose.position.y = float(dt_box_lidar[i][1])
                marker.pose.position.z = float(dt_box_lidar[i][2])
                
                q = self.yaw2quaternion(float(dt_box_lidar[i][8]))
                marker.pose.orientation.x = q[1]
                marker.pose.orientation.y = q[2]
                marker.pose.orientation.z = q[3]
                marker.pose.orientation.w = q[0]
                
                marker.scale.x = float(dt_box_lidar[i][4])
                marker.scale.y = float(dt_box_lidar[i][3])
                marker.scale.z = float(dt_box_lidar[i][5])
                
                marker.color.a = 0.7  # Alpha
                if int(types[i]) == 9:
                    marker.color.r = 1.0
                    marker.color.g = 1.0
                    marker.color.b = 1.0
                elif int(types[i]) == 8:
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                elif int(types[i]) == 7:
                    marker.color.r = 0.0
                    marker.color.g = 0.0
                    marker.color.b = 1.0
                elif int(types[i]) == 6:
                    marker.color.r = 1.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                elif int(types[i]) == 5:
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 1.0
                elif int(types[i]) == 4:
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 1.0
                elif int(types[i]) == 3:
                    marker.color.r = 0.5
                    marker.color.g = 0.5
                    marker.color.b = 0.5
                elif int(types[i]) == 2:
                    marker.color.r = 0.5
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                elif int(types[i]) == 1:
                    marker.color.r = 0.0
                    marker.color.g = 0.5
                    marker.color.b = 0.0
                elif int(types[i]) == 0:
                    marker.color.r = 0.0
                    marker.color.g = 0.0
                    marker.color.b = 0.5
                else:
                    marker.color.r = 1.0
                    marker.color.g = 1.0
                    marker.color.b = 1.0  # Default color white for unknown types
                
                marker.lifetime = Duration(sec=0, nanosec=int(0.2 * 1e9))  # Set lifetime to 0.2 seconds

                arr_bbox.markers.append(marker)

                # ---------- AUTOWARE MSG FORMAT -----------------
                # bbox = PredictedObject()
                # classification = ObjectClassification()

                # classification.probability = 1.0
                # if int(types[i]) == 9:
                #     classification.label = ObjectClassification.UNKNOWN
                # elif int(types[i]) == 8:
                #     classification.label = ObjectClassification.PEDESTRIAN
                # elif int(types[i]) == 7:
                #     classification.label = ObjectClassification.UNKNOWN
                # elif int(types[i]) == 6:
                #     classification.label = ObjectClassification.BICYCLE
                # elif int(types[i]) == 5:
                #     classification.label = ObjectClassification.UNKNOWN
                # elif int(types[i]) == 4:
                #     classification.label = ObjectClassification.TRAILER
                # elif int(types[i]) == 3:
                #     classification.label = ObjectClassification.BUS
                # elif int(types[i]) == 2:
                #     classification.label = ObjectClassification.UNKNOWN
                # elif int(types[i]) == 1:
                #     classification.label = ObjectClassification.TRUCK
                # elif int(types[i]) == 0:
                #     classification.label = ObjectClassification.CAR

                # bbox.shape.type = Shape.BOUNDING_BOX
                # bbox.shape.dimensions.x = float(dt_box_lidar[i][4])
                # bbox.shape.dimensions.y = float(dt_box_lidar[i][3])
                # bbox.shape.dimensions.z = float(dt_box_lidar[i][5])

                # q = self.yaw2quaternion(float(dt_box_lidar[i][8]))
                # bbox.kinematics.initial_pose_with_covariance.pose.position.x = float(dt_box_lidar[i][0])
                # bbox.kinematics.initial_pose_with_covariance.pose.position.y = float(dt_box_lidar[i][1])
                # bbox.kinematics.initial_pose_with_covariance.pose.position.z = float(dt_box_lidar[i][2])
                # bbox.kinematics.initial_pose_with_covariance.pose.orientation.x = q[1]
                # bbox.kinematics.initial_pose_with_covariance.pose.orientation.y = q[2]
                # bbox.kinematics.initial_pose_with_covariance.pose.orientation.z = q[3]
                # bbox.kinematics.initial_pose_with_covariance.pose.orientation.w = q[0]
                

                # bbox.existence_probability = float(scores[i])
                # bbox.classification.append(classification)

                # arr_bbox.objects.append(bbox)

                # ------------- JSK MSG FORMAT --------------------
                # bbox = BoundingBox()
                # bbox.header.frame_id = msg.header.frame_id
                # bbox.header.stamp = self.get_clock().now().to_msg()
                # q = self.yaw2quaternion(float(dt_box_lidar[i][8]))
                # bbox.pose.orientation.x = q[1]
                # bbox.pose.orientation.y = q[2]
                # bbox.pose.orientation.z = q[3]
                # bbox.pose.orientation.w = q[0]           
                # bbox.pose.position.x = float(dt_box_lidar[i][0])
                # bbox.pose.position.y = float(dt_box_lidar[i][1])
                # bbox.pose.position.z = float(dt_box_lidar[i][2])
                # bbox.dimensions.x = float(dt_box_lidar[i][4])
                # bbox.dimensions.y = float(dt_box_lidar[i][3])
                # bbox.dimensions.z = float(dt_box_lidar[i][5])
                # bbox.value = scores[i]
                # bbox.label = int(types[i])
                # arr_bbox.boxes.append(bbox)
        print("total callback time: ", 1/(default_timer() - t_t))
        # arr_bbox.header.frame_id = msg.header.frame_id
        # arr_bbox.header.stamp = msg.header.stamp

        #if len(arr_bbox.objects) > 0:
        if len(arr_bbox.markers) > 0:
            self.pub_arr_bbox.publish(arr_bbox)
            #arr_bbox.objects = []
            arr_bbox.markers = []
        else:
            #arr_bbox.objects = []
            arr_bbox.markers = []
            self.pub_arr_bbox.publish(arr_bbox)


def main(args=None):
    rclpy.init(args=args)
    cone_detector_node = ConeDetectorRosNode()
    rclpy.spin(cone_detector_node)
    cone_detector_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

