from timeit import default_timer

import numpy as np
from pyquaternion import Quaternion
from mmdet3d.apis import init_model, inference_detector

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import ros2_numpy
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration


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
                    ("/cone_detector/model_config", '/home/dobayt/git/mmdetection3d/configs/centerpoint/centerpoint_pillar02_second_secfpn_head-dcn_8xb4-cyclic-20e_nus-3d.py'),
                    ("/cone_detector/model_checkpoints", '/home/dobayt/ros2_ws/src/formula_student_packages/lidar_centerpoint_detector/lidar_centerpoint_detector/models/ckpt_centerpoint_nuscenes/centerpoint_02pillar_second_secfpn_dcn_4x8_cyclic_20e_nus_20220811_045458-808e69ad.pth'),
                ],
            )

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
    
    def get_xyz_points(self, cloud_array: dict, remove_nans=True, dtype=float) -> np.ndarray:
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

    def run_detector(self, input_pcl: np.array) -> np.ndarray:
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
        
        pred_results, _ = inference_detector(self.model, input_pcl)

        pred_dict_filtered = remove_low_score_nu(pred_results.to_dict()["pred_instances_3d"])

        return pred_dict_filtered['scores_3d'], pred_dict_filtered['bboxes_3d'], pred_dict_filtered['labels_3d']

    # CALLBACKS
    def lidar_callback(self, msg):
        arr_bbox = MarkerArray()

        t1 = default_timer()
        msg_cloud = ros2_numpy.point_cloud2.pointcloud2_to_array(msg)
        np_p = self.get_xyz_points(msg_cloud, True)
        scores, dt_box_lidar, types = self.run_detector(np_p)

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
