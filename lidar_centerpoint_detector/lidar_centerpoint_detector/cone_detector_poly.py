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

# def remove_low_score_cone(image_anno: dict) -> dict:
#     img_filtered_annotations = {}
#     label_preds_ = image_anno["labels_3d"].detach().cpu().numpy()
#     scores_ = image_anno["scores_3d"].detach().cpu().numpy()

#     cone_yellow = get_annotations_indices(0, 0.35, label_preds_, scores_)
#     cone_blue = get_annotations_indices(1, 0.35, label_preds_, scores_)
#     #cone_orange = get_annotations_indices(2, 0.35, label_preds_, scores_)
#     cone_big = get_annotations_indices(2, 0.1, label_preds_, scores_)
    
#     for key in image_anno.keys():
#         if key == 'polyln_3d':
#             continue
#         img_filtered_annotations[key] = (
#             image_anno[key].detach().cpu().numpy()[
#                             cone_yellow +
#                             cone_blue +
#                             #cone_orange +
#                             cone_big
#                             ])

#     return img_filtered_annotations

def remove_low_score_cone(image_anno: dict) -> dict:
    img_filtered_annotations = {}
    label_preds_ = image_anno["labels_3d"].detach().cpu().numpy()
    scores_ = image_anno["scores_3d"].detach().cpu().numpy()

    cone_yellow = get_annotations_indices(0, 0.35, label_preds_, scores_)
    cone_blue = get_annotations_indices(1, 0.35, label_preds_, scores_)
    #cone_orange = get_annotations_indices(2, 0.35, label_preds_, scores_)
    cone_big = get_annotations_indices(2, 0.1, label_preds_, scores_)
    
    for key in image_anno.keys():
        # keep polylines; only skip non-per-instance meta
        if key in ('box_type_3d', 'poly_type_3d'):
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
                    ("/cone_detector/model_config", '/home/A_Balint/mmdetection3d/configs/centerpoint/centerpoint_pillar02_second_secfpn_head-circlenms_8xb4-cyclic-20e_cone-3d-curve.py'),
                    ("/cone_detector/model_checkpoints", '/home/A_Balint/mmdetection3d/work_dirs/centerpoint_pillar02_second_secfpn_8xb4-cyclic-20e_cone-3d-curve/epoch_120.pth'),
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

        return pred_dict_filtered['scores_3d'], pred_dict_filtered['polyln_3d'], pred_dict_filtered['labels_3d']
    
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


    def lidar_callback(self, msg):
        arr_bbox = MarkerArray()

        t1 = default_timer()
        msg_cloud = ros2_numpy.point_cloud2.pointcloud2_to_array(msg)
        np_p = self.get_xyz_points(msg_cloud, True)
        scores, dt_poly_lidar_no_tf, types = self.run_detector(np_p)  # may be boxes or polylines

        # common TF
        H = np.eye(4, dtype=np.float32)
        H[:3, :3] = rot.from_euler("xyz", [0.0, 0.0, np.pi]).as_matrix()
        H[:3, 3] = [0.466, 0.0, 0.849]
        H = np.linalg.inv(H)

        def _tf_xyz_pts(xyz):  # xyz: (M,3)
            pts = np.hstack((xyz, np.ones((xyz.shape[0], 1), dtype=xyz.dtype)))
            out = (pts @ H.T)[:, :3]
            return out.astype(np.float32)

        if scores.size != 0:
            feat_dim = dt_poly_lidar_no_tf.shape[1] if dt_poly_lidar_no_tf.ndim == 2 else 0
            # geoms_no_tf[i] encodes concatenated (x,y,z, x,y,z, ...) in VEH frame
            from geometry_msgs.msg import Point

            n_ctrl = feat_dim // 3
            for i in range(scores.size):
                flat = dt_poly_lidar_no_tf[i]
                # guard
                if flat.ndim != 1 or flat.size < 6 or flat.size % 3 != 0:
                    continue
                ctrl_vehicle = flat.reshape(n_ctrl, 3)
                ctrl_sensor = _tf_xyz_pts(ctrl_vehicle)

                marker = Marker()
                marker.header.frame_id = msg.header.frame_id
                marker.header.stamp = msg.header.stamp
                marker.ns = "polylines"
                marker.id = i
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD

                # width of the line
                marker.scale.x = 0.08
                marker.color.a = 0.95
                if int(types[i]) == 1:
                    marker.color.r, marker.color.g, marker.color.b = (0.0, 0.0, 1.0)   # blue
                elif int(types[i]) == 0:
                    marker.color.r, marker.color.g, marker.color.b = (1.0, 1.0, 0.0)   # yellow
                elif int(types[i]) == 2:
                    marker.color.r, marker.color.g, marker.color.b = (1.0, 0.65, 0.0) # orange/big
                else:
                    marker.color.r, marker.color.g, marker.color.b = (1.0, 1.0, 1.0)

                marker.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in ctrl_sensor]
                marker.lifetime = Duration(sec=0, nanosec=int(0.2 * 1e9))
                arr_bbox.markers.append(marker)

        print("total callback time: ", 1/(default_timer() - t1))
        self.pub_arr_bbox.publish(arr_bbox)
        arr_bbox.markers = []


def main(args=None):
    rclpy.init(args=args)
    cone_detector_node = ConeDetectorRosNode()
    rclpy.spin(cone_detector_node)
    cone_detector_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

