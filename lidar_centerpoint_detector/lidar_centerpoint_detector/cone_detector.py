from timeit import default_timer

import torch
import mmengine
import numpy as np
from pyquaternion import Quaternion
from mmdet3d.apis import init_model, inference_detector
#from mmdeploy.utils import get_input_shape, load_config # TODO: after deployment is working
from scipy.spatial.transform import Rotation as rot
#from mmdeploy.apis.utils import build_task_processor
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import ros2_numpy
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration


def get_annotations_indices(types, thresh, label_preds, scores):
                indexs = []
                annotation_indices = []
                for i in range(len(label_preds)):
                    if label_preds[i] == types:
                        indexs.append(i)
                for index in indexs:
                    if scores[index] >= thresh:
                        annotation_indices.append(index)
                return annotation_indices

def remove_low_score_nu(image_anno: dict) -> dict:
    img_filtered_annotations = {}
    label_preds_ = image_anno["labels_3d"].detach().cpu().numpy()
    scores_ = image_anno["scores_3d"].detach().cpu().numpy()
    
    # car_indices =                  get_annotations_indices(0, 0.4, label_preds_, scores_)
    # truck_indices =                get_annotations_indices(1, 0.4, label_preds_, scores_)
    # construction_vehicle_indices = get_annotations_indices(2, 0.4, label_preds_, scores_)
    # bus_indices =                  get_annotations_indices(3, 0.3, label_preds_, scores_)
    # trailer_indices =              get_annotations_indices(4, 0.4, label_preds_, scores_)
    # barrier_indices =              get_annotations_indices(5, 0.4, label_preds_, scores_)
    # motorcycle_indices =           get_annotations_indices(6, 0.15, label_preds_, scores_)
    # bicycle_indices =              get_annotations_indices(7, 0.15, label_preds_, scores_)
    # pedestrain_indices =           get_annotations_indices(8, 0.1, label_preds_, scores_)
    traffic_cone_indices =           get_annotations_indices(9, 0.4, label_preds_, scores_)
    
    for key in image_anno.keys():
        if key == 'box_type_3d':
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
            image_anno[key].detach().cpu().numpy()[
                            traffic_cone_indices
                            ])

    return img_filtered_annotations

def remove_low_score_cone(image_anno: dict) -> dict:
    img_filtered_annotations = {}
    label_preds_ = image_anno["labels_3d"].detach().cpu().numpy()
    scores_ = image_anno["scores_3d"].detach().cpu().numpy()

    cone_yellow = get_annotations_indices(0, 0.35, label_preds_, scores_)
    cone_blue = get_annotations_indices(1, 0.35, label_preds_, scores_)
    #cone_orange = get_annotations_indices(2, 0.35, label_preds_, scores_)
    cone_big = get_annotations_indices(2, 0.1, label_preds_, scores_)
    
    for key in image_anno.keys():
        if key == 'box_type_3d':
            continue
        img_filtered_annotations[key] = (
            image_anno[key].detach().cpu().numpy()[
                            cone_yellow +
                            cone_blue +
                            #cone_orange +
                            cone_big
                            ])

    return img_filtered_annotations


class ConeDetectorRosNode(Node):
    def __init__(self) -> None:
        super().__init__("cone_detector_node")
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=1
        )

        # declare rosparams
        if not self.has_parameter("/cone_detector/lidar_input_topic"):
            self.declare_parameters(
                namespace="",
                parameters=[
                    ("/cone_detector/lidar_input_topic", '/points'),
                    ("/cone_detector/model_config", '/home/dobayt/git/mmdetection3d/configs/centerpoint/centerpoint_pillar02_second_secfpn_head-dcn_8xb4-cyclic-20e_cone-3d.py'),
                    ("/cone_detector/model_checkpoints", '/home/dobayt/ros2_ws/src/formula_student_packages/lidar_centerpoint_detector/lidar_centerpoint_detector/models/ckpt_centerpoint_conescenes/epoch_20_3pts_51_zala_cones.pth'),
                    # ("/cone_detector/model_config", '/home/dobayt/git/mmdetection3d/configs/centerpoint/centerpoint_pillar02_second_secfpn_head-circlenms_8xb4-cyclic-20e_cone-3d.py'),
                    # ("/cone_detector/model_checkpoints", '/home/dobayt/git/mmdetection3d/work_dirs/centerpoint_pillar02_second_secfpn_head-circlenms_8xb4-cyclic-20e_cone-3d/transfer_minpts3_20e/epoch_20.pth'),
                    # ("/cone_detector/model_deploy_config", '/home/dobayt/git/mmdeploy/configs/mmdet3d/voxel-detection/voxel-detection_tensorrt_dynamic-nus-20x5.py'),
                    # ("/cone_detector/model_trt_file", '/home/dobayt/git/mmdeploy_model/centerpoint_cone_trt/end2end.engine'),
                    ("/cone_detector/dataset", 'cone'), # only 'cone' or 'nus' are supported
                ],
            )
        # deployed inference
        # deploy_cfg, model_cfg = load_config(self.get_parameter("/cone_detector/model_deploy_config").value, self.get_parameter("/cone_detector/model_config").value)
        # self.task_processor = build_task_processor(model_cfg, deploy_cfg, device='cuda:0')
        # self.model = self.task_processor.build_backend_model(
        # [self.get_parameter("/cone_detector/model_trt_file").value], self.task_processor.update_data_preprocessor)
        # self.input_shape = get_input_shape(deploy_cfg)
        # standard inference
        self.model = init_model(self.get_parameter("/cone_detector/model_config").value, self.get_parameter("/cone_detector/model_checkpoints").value, device="cuda:0")

        self.pcl_sub = self.create_subscription(
            PointCloud2, self.get_parameter("/cone_detector/lidar_input_topic").value, self.lidar_callback, qos_profile #10
        )

        self.pub_arr_bbox = self.create_publisher(MarkerArray, "/detected_cones", 1)
        
        # example log
        self.get_logger().info(f"Loading model checkpoint: {self.get_parameter('/cone_detector/model_checkpoints').value}")

    # HELPER FUNCTIONS
    def yaw2quaternion(self, yaw: float) -> Quaternion:
        return Quaternion(axis=[0,0,1], radians=yaw)
    
    def transform_points(self, pcl: np.array, H: np.array) -> np.array:
        """
        Transforms a 3D point cloud with intensity values using a homogeneous transformation matrix.

        Parameters:
            pcl (np.ndarray): An Nx4 array representing the 3D point cloud (x, y, z, intensity).
            H (np.ndarray): A 4x4 homogeneous transformation matrix.

        Returns:
            np.ndarray: The transformed 3D point cloud with intensity values preserved.
        """
        intensity = pcl[:, 3:].reshape(-1,pcl.shape[1]-3)
        pcl = pcl[:, :3]
        pcl = np.hstack((pcl, np.ones((pcl.shape[0],1))))

        tranformed_pcl = pcl @ H.T
        tranformed_pcl = tranformed_pcl[:,:3]
        tranformed_pcl = np.hstack((tranformed_pcl, intensity))

        return tranformed_pcl
    
    def get_xyz_points(self, cloud_array: dict, remove_nans=True, dtype=float) -> np.ndarray:
        '''
        '''
        # if remove_nans:
        mask = np.isfinite(cloud_array['x']) & np.isfinite(cloud_array['y']) & np.isfinite(cloud_array['z'])
        cloud_array = cloud_array[mask]

        # shape depends on the num_dim the model was trained on
        if self.get_parameter("/cone_detector/dataset").value == 'cone':
            points = np.zeros(cloud_array.shape + (4,), dtype=dtype)
        elif self.get_parameter("/cone_detector/dataset").value == 'nus':
            points = np.zeros(cloud_array.shape + (5,), dtype=dtype)

        points[...,0] = cloud_array['x']
        points[...,1] = cloud_array['y']
        points[...,2] = cloud_array['z']
        points[...,3] = cloud_array['intensity']

        H = np.eye(4, dtype=np.float32)
        H[:3, :3] = rot.from_euler("xyz", [0.0, 0.0, np.pi]).as_matrix()
        H[:3, 3] = [0.466, 0.0, 0.849]
        # transform pcl from sensor coord sys to vehicle coord sys
        tf_points = self.transform_points(points, H).astype(np.float32)

        return tf_points

    def run_detector(self, input_pcl: np.array) -> np.ndarray:
        pred_results, _ = inference_detector(self.model, input_pcl)

        if self.get_parameter("/cone_detector/dataset").value == 'cone':
            pred_dict_filtered = remove_low_score_cone(pred_results.to_dict()["pred_instances_3d"])
        elif self.get_parameter("/cone_detector/dataset").value == 'nus':
            pred_dict_filtered = remove_low_score_nu(pred_results.to_dict()["pred_instances_3d"])

        return pred_dict_filtered['scores_3d'], pred_dict_filtered['bboxes_3d'], pred_dict_filtered['labels_3d']
    
    def run_detector_trt(self, input_pcl: np.array) -> np.ndarray:
        t1 = default_timer()
        model_inputs, _ = self.task_processor.create_input(input_pcl, self.input_shape)
        print(f"Input creation took: {default_timer() - t1}")
        t1 = default_timer()
        with torch.no_grad():
            pred_results = self.model.test_step(model_inputs)[0]
        print(f"Inference took: {default_timer() - t1}")

        if self.get_parameter("/cone_detector/dataset").value == 'cone':
            pred_dict_filtered = remove_low_score_cone(pred_results.to_dict()["pred_instances_3d"])
        elif self.get_parameter("/cone_detector/dataset").value == 'nus':
            pred_dict_filtered = remove_low_score_nu(pred_results.to_dict()["pred_instances_3d"])

        return pred_dict_filtered['scores_3d'], pred_dict_filtered['bboxes_3d'], pred_dict_filtered['labels_3d']

    # CALLBACKS
    def lidar_callback(self, msg):
        arr_bbox = MarkerArray()

        t1 = default_timer()
        msg_cloud = ros2_numpy.point_cloud2.pointcloud2_to_array(msg)
        np_p = self.get_xyz_points(msg_cloud, True)
        scores, dt_box_lidar_no_tf, types = self.run_detector(np_p)

        H = np.eye(4, dtype=np.float32)
        H[:3, :3] = rot.from_euler("xyz", [0.0, 0.0, np.pi]).as_matrix()
        H[:3, 3] = [0.466, 0.0, 0.849]
        H = np.linalg.inv(H)
        # transform bbox from veh coord sys to sensor coord sys
        dt_box_lidar = self.transform_points(dt_box_lidar_no_tf, H).astype(np.float32)
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
                
                marker.color.a = 0.85  # Alpha
                if int(types[i]) == 1:
                    marker.color.r = 0.0
                    marker.color.g = 0.0
                    marker.color.b = 1.0
                elif int(types[i]) == 0:
                    marker.color.r = 1.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                elif int(types[i]) == 3:
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                elif int(types[i]) == 2:
                    marker.color.r = 1.0
                    marker.color.g = 0.65
                    marker.color.b = 0.0
                # elif int(types[i]) == 9:
                #     marker.color.r = 1.0
                #     marker.color.g = 1.0
                #     marker.color.b = 1.0
                # elif int(types[i]) == 8:
                #     marker.color.r = 0.0
                #     marker.color.g = 1.0
                #     marker.color.b = 0.0
                # elif int(types[i]) == 7:
                #     marker.color.r = 0.0
                #     marker.color.g = 0.5
                #     marker.color.b = 1.0
                # elif int(types[i]) == 6:
                #     marker.color.r = 1.0
                #     marker.color.g = 1.0
                #     marker.color.b = 0.0
                # elif int(types[i]) == 5:
                #     marker.color.r = 1.0
                #     marker.color.g = 0.0
                #     marker.color.b = 1.0
                # elif int(types[i]) == 4:
                #     marker.color.r = 0.0
                #     marker.color.g = 1.0
                #     marker.color.b = 1.0
                else:
                    marker.color.r = 1.0
                    marker.color.g = 1.0
                    marker.color.b = 1.0  # Default color white for unknown types
                
                marker.lifetime = Duration(sec=0, nanosec=int(0.2 * 1e9))  # Set lifetime to 0.2 seconds

                arr_bbox.markers.append(marker)
        print("total callback time: ", 1/(default_timer() - t1))
        if len(arr_bbox.markers) > 0:
            self.pub_arr_bbox.publish(arr_bbox)
            arr_bbox.markers = []
        else:
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

