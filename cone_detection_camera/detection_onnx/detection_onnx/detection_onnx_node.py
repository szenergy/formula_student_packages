#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header, Bool, String, UInt32, Float32MultiArray, MultiArrayDimension
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from rclpy.qos import qos_profile_sensor_data
import cv2
import numpy as np
from yolov8 import YOLOv8

class ObjectDetectorRosNode(Node):

    def __init__(self):
        super().__init__('parking_place_detector')
        self.declare_parameter('detection_rate_hz', 0.1)
        self.declare_parameter('onnx_file_path', 'onnx_file_path/model2.onnx')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.4)

        self.img_pub = self.create_publisher(Image, "detect_cone_img", 1)
        self.array_pub = self.create_publisher(Float32MultiArray, "cone_coordinates", 1)
        self.bridge = CvBridge()
        self.subscribe_topic = self.create_subscription(Image, '/zed2i/zed_node/right/image_rect_color', self.img_callback, qos_profile_sensor_data)

        self.detection_counter = 0
        self.previous_detection_count = 0

        self.previous_state = ""
        self.infer_img = None
        self.detection_rate_hz = self.get_parameter('detection_rate_hz').get_parameter_value().double_value

        model_path = self.get_parameter('onnx_file_path').get_parameter_value().string_value
        self.yolov8_detector = YOLOv8(model_path, conf_thres=self.get_parameter('confidence_threshold').get_parameter_value().double_value, iou_thres=self.get_parameter('iou_threshold').get_parameter_value().double_value)

        self.timer = self.create_timer(0.01, self.inference)

    def img_callback(self, data):
        try:
            self.infer_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv2.imshow("Input Image", self.infer_img)
        except CvBridgeError as e:
            self.get_logger().error(str(e))

    def inference(self):
        if self.infer_img is not None:
            boxes, scores, class_ids = self.yolov8_detector(self.infer_img)
            if 1 == 1:
                img = self.yolov8_detector.draw_detections(self.infer_img)
                detection_data = []
                for i in range(len(boxes)):
                    box = boxes[i]
                    object_type = 0.0  #Def
                    if class_ids[i] == 4:  #Y
                        object_type = 1.0
                    elif class_ids[i] == 0:  #B
                        object_type = 2.0
                    elif class_ids[i] in [1, 2]:  #o
                        object_type = 3.0

                    detection_data.extend([
                        object_type, float(box[0]), float(box[1]), float(box[2] - box[0]), 0.0                           
                    ])#0. element: detected object type,  1. element: X coordinate, 2. element: Y coordinate, 3. element: width, 4. element: always 0.0 

                if detection_data:
                    parking_array = Float32MultiArray()
                    parking_array.data = detection_data
                    self.array_pub.publish(parking_array)

                self.img_pub.publish(self.bridge.cv2_to_imgmsg(img))
                cv2.imshow("Detection Output", img)

            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectorRosNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
