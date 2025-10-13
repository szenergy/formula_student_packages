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
from scipy import interpolate

import numpy as np

def catmull_rom_spline(points: np.ndarray, samples_per_seg: int = 20, alpha: float = 0.5) -> np.ndarray:
    """
    Sample a Catmull–Rom spline through `points` (N,3).
    - `alpha`=0.5 is centripetal (avoids loops/overshoot on sharp turns).
    - Returns an array (M,3) of sampled points.
    """
    pts = np.asarray(points, dtype=np.float32)
    n = len(pts)
    if n < 2:
        return pts
    if n == 2:
        # not enough points: just linearly sample
        t = np.linspace(0, 1, samples_per_seg, dtype=np.float32)
        return (1 - t)[:, None] * pts[0] + t[:, None] * pts[1]

    # duplicate endpoints to make end segments well-defined
    P = np.vstack([pts[0], pts, pts[-1]])  # (n+2,3)

    # compute knot parameterization
    def tj(ti, pi, pj):
        return ti + (np.linalg.norm(pj - pi) ** alpha)
    T = [0.0]
    for i in range(1, len(P)):
        T.append(tj(T[-1], P[i-1], P[i]))
    T = np.array(T, dtype=np.float32)

    out = []
    for i in range(1, len(P) - 2):
        p0, p1, p2, p3 = P[i-1], P[i], P[i+1], P[i+2]
        t0, t1, t2, t3 = T[i-1], T[i], T[i+1], T[i+2]

        # local parameter range
        ts = np.linspace(t1, t2, samples_per_seg, dtype=np.float32)
        # tangent vectors (Finite differences in CR)
        m1 = (p2 - p0) / (t2 - t0)
        m2 = (p3 - p1) / (t3 - t1)

        # Hermite basis on [t1, t2] remapped to u in [0,1]
        u = (ts - t1) / (t2 - t1)
        u2 = u * u
        u3 = u2 * u
        h00 =  2*u3 - 3*u2 + 1
        h10 =    u3 - 2*u2 + u
        h01 = -2*u3 + 3*u2
        h11 =    u3 -   u2

        seg = (h00[:, None] * p1 +
               h10[:, None] * (t2 - t1) * m1 +
               h01[:, None] * p2 +
               h11[:, None] * (t2 - t1) * m2)
        out.append(seg)

    return np.vstack(out)



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

def remove_low_score_poly(poly_anno: dict) -> dict:
    """
    Filter out low-confidence lane polyline predictions per class.

    Args:
        poly_anno (dict): Dictionary containing polyline predictions with keys:
            - "polyln_3d" or "polylines_3d": (N, D)
            - "labels_3d": (N,)
            - "scores_3d": (N,)

    Returns:
        dict: Filtered polyline annotations.
    """

    filtered_poly_anno = {}

    # --- Convert tensors to numpy ---
    labels_ = poly_anno["labels_3d"].detach().cpu().numpy()
    scores_ = poly_anno["scores_3d"].detach().cpu().numpy()

    # --- Class-dependent thresholds ---
    # You can tune these values depending on detection quality
    Left_Lane_1  = get_annotations_indices(0, 0.35, labels_, scores_)
    Right_Lane_1 = get_annotations_indices(1, 0.35, labels_, scores_)
    Left_Lane_2  = get_annotations_indices(2, 0.30, labels_, scores_)
    Right_Lane_2 = get_annotations_indices(3, 0.30, labels_, scores_)

    selected_indices = Left_Lane_1 + Right_Lane_1 + Left_Lane_2 + Right_Lane_2

    # --- Filter all relevant instance-level fields ---
    for key in poly_anno.keys():
        if key in ("box_type_3d", "poly_type_3d"):
            continue

        data = poly_anno[key]
        if isinstance(data, torch.Tensor):
            data = data.detach().cpu().numpy()

        if data is None or len(data) == 0:
            continue

        try:
            filtered_poly_anno[key] = data[selected_indices]
        except Exception:
            # In case of non-matching lengths (e.g., metadata)
            filtered_poly_anno[key] = data

    return filtered_poly_anno


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
                    ("/cone_detector/dataset", 'poly'), # only 'cone' or 'nus' are supported
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
        else:
            points = np.zeros(cloud_array.shape + (4,), dtype=dtype)

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
        elif self.get_parameter("/cone_detector/dataset").value == 'poly':
            pred_dict_filtered = remove_low_score_poly(pred_results.to_dict()["pred_instances_3d"])
        
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
        """Visualize predicted lane polylines (B-spline or XYZ) in RViz."""
        from geometry_msgs.msg import Point
        from scipy.interpolate import make_interp_spline

        arr_bbox = MarkerArray()
        t1 = default_timer()

        # Convert ROS point cloud to numpy
        msg_cloud = ros2_numpy.point_cloud2.pointcloud2_to_array(msg)
        np_p = self.get_xyz_points(msg_cloud, True)

        # Run detector: returns scores, polylines, labels
        scores, dt_poly_lidar_no_tf, types = self.run_detector(np_p)
        if scores.size == 0:
            return  # nothing to visualize

        # Transformation setup
        H = np.eye(4, dtype=np.float32)
        H[:3, :3] = rot.from_euler("xyz", [0.0, 0.0, np.pi]).as_matrix()
        H[:3, 3] = [0.466, 0.0, 0.849]
        H = np.linalg.inv(H)

        def _tf_xyz_pts(xyz: np.ndarray) -> np.ndarray:
            """Apply homogeneous transformation (vehicle -> sensor frame)."""
            pts = np.hstack((xyz, np.ones((xyz.shape[0], 1), dtype=xyz.dtype)))
            return (pts @ H.T)[:, :3].astype(np.float32)

        # Define lane class colors
        lane_colors = {
            0: (1.0, 1.0, 0.0),  # Left_Lane_1 - yellow
            1: (0.0, 0.0, 1.0),  # Right_Lane_1 - blue
            2: (1.0, 0.5, 0.0),  # Left_Lane_2 - orange
            3: (0.0, 1.0, 0.0),  # Right_Lane_2 - green
        }

        # Main loop: per predicted polyline
        for i in range(scores.size):
            flat = dt_poly_lidar_no_tf[i]

            # Validate input
            flat = np.array(flat.cpu().numpy()).squeeze()
            if flat.ndim != 1 or flat.size < 6:
                continue

            # Decode polyline
            if flat.size == 38:
                # B-spline encoded format
                num_cp_xy, num_cp_xz, num_endpoints = 8, 8, 2
                cp_xy = flat[:num_cp_xy * 2].reshape(-1, 2)
                cp_xz = flat[num_cp_xy * 2:num_cp_xy * 2 + num_cp_xz * 2].reshape(-1, 2)

                num_points = 100  # number of interpolated points

                try:
                    # Interpolate with automatic degree selection
                    t = np.linspace(0, 1, len(cp_xy))
                    x_new = make_interp_spline(t, cp_xy[:, 0], k=min(3, len(cp_xy) - 1))(np.linspace(0, 1, num_points))
                    y_new = make_interp_spline(t, cp_xy[:, 1], k=min(3, len(cp_xy) - 1))(np.linspace(0, 1, num_points))

                    t_z = np.linspace(0, 1, len(cp_xz))
                    z_new = make_interp_spline(t_z, cp_xz[:, 1], k=min(3, len(cp_xz) - 1))(np.linspace(0, 1, num_points))

                    ctrl_vehicle = np.column_stack([x_new, y_new, z_new])
                except Exception as e:
                    self.get_logger().warn(f"[Spline fallback] Using linear interpolation: {e}")
                    x_new = np.linspace(cp_xy[0, 0], cp_xy[-1, 0], num_points)
                    y_new = np.linspace(cp_xy[0, 1], cp_xy[-1, 1], num_points)
                    z_new = np.linspace(cp_xz[0, 1], cp_xz[-1, 1], num_points)
                    ctrl_vehicle = np.column_stack([x_new, y_new, z_new])
            else:
                # Already a polyline in 3D (x, y, z, ...)
                n_ctrl = flat.size // 3
                ctrl_vehicle = flat.reshape(n_ctrl, 3)

            # Apply transform to ROS frame
            ctrl_sensor = _tf_xyz_pts(ctrl_vehicle)

            # Create RViz Marker
            marker = Marker()
            marker.header.frame_id = msg.header.frame_id
            marker.header.stamp = msg.header.stamp
            lane_type = int(types[i])

            marker.ns = f"lane_class_{lane_type}"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.07
            marker.color.a = 0.95

            # Color by lane class
            marker.color.r, marker.color.g, marker.color.b = lane_colors.get(lane_type, (1.0, 1.0, 1.0))

            # Fade opacity by score
            confidence = float(scores[i])
            marker.color.a = min(1.0, 0.5 + confidence * 0.5)

            # Add spline points to marker
            marker.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in ctrl_sensor]
            marker.lifetime = Duration(sec=0, nanosec=int(0.3 * 1e9))
            arr_bbox.markers.append(marker)

        # Publish all markers
        elapsed = (default_timer() - t1) * 1000
        print(f"[lidar_callback] Time: {elapsed:.2f} ms  ({1000/elapsed:.1f} FPS)")
        self.pub_arr_bbox.publish(arr_bbox)
        arr_bbox.markers.clear()




def main(args=None):
    rclpy.init(args=args)
    cone_detector_node = ConeDetectorRosNode()
    rclpy.spin(cone_detector_node)
    cone_detector_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

