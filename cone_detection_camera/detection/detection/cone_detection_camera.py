#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension

import time

import cv2
import numpy as np
from matplotlib import pyplot as plt

import os
import time

import argparse

class ConeDetector(Node):
    def __init__(self, weight, config, display, save, plot, countmsg):
        super().__init__('cone_detector_camera')
        # ----- Init camera data and thresholds -----

        self.scriptDir = os.path.dirname(os.path.realpath(__file__))
        # self.frame_number = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.number_of_detections = []
        self.fps_counter = []
        self.confThreshold = 0.5
        self.nmsThreshold = 0.3
        self.frame_counter = 0

        self.br = CvBridge()

        # ----- Init ROS data -----
        self.zed_subscriber = self.create_subscription(Image, '/zed2i/zed_node/right/image_rect_color', self.imageCallback, qos_profile_sensor_data)
        self.cone_coord_publisher = self.create_publisher(Float32MultiArray, 'cone_coordinates', 1)
        self.cone_image_publisher = self.create_publisher(Image, 'detect_cone_img', 1)
        
        self.class_colors = {'YELLOW':(0, 255, 255), 'SMALL_ORANGE':(0, 100, 255), 'OTHER':(255, 255, 0),'BLUE':(255, 0, 0)}

        # ----- Target width and height given to neural network -----
        self.wT, self.hT = (256, 256)

        # ----- Get classes -----
        classesFile = self.scriptDir+'/resources/class.names'
        self.classNames = []
        with open(classesFile, 'rt') as f:
            self.classNames = f.read().rstrip('\n').split('\n')

        # ----- Get config and weights file and load them to DNN -----
        self.modelConfiguration = config
        self.modelWeights = weight

        self.net = cv2.dnn.readNetFromDarknet(self.modelConfiguration, self.modelWeights)

        # ----- Use CPU -----
        #net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        #net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

        # ----- Use GPU's CUDA -----
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)

        if countmsg:
            self.sentCones = []
            self.countCones = 0

    # ------------------------------ Implementing methods ------------------------------
    # ***** Image Callback for ZED camera *****
    def imageCallback(self, msg):
        # ------------------------------ GET IMAGE DATA ------------------------------
        
        frame = self.br.imgmsg_to_cv2(msg)

        frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2RGB)

        masked_frame = self.maskImage(frame)
        # cv2.imshow("Masked image", masked_frame)
        blob = cv2.dnn.blobFromImage(masked_frame, 1/255, (self.wT, self.hT), [0,0,0], 1, crop=False)
        self.net.setInput(blob)

        layerNames = self.net.getLayerNames()
        #print(layerNames)
        #print(self.net.getUnconnectedOutLayers())
        try:
            outputNames = [layerNames[i[0]-1] for i in self.net.getUnconnectedOutLayers()]
        except IndexError:
            outputNames = [layerNames[i-1] for i in self.net.getUnconnectedOutLayers()]
        #print(outputNames)

        outputs = self.net.forward(outputNames)

        #print(outputs[0].shape)
        #print(outputs[1].shape)
        #print(outputs[2].shape)

        
        data = self.findObjects(outputs, masked_frame)

        # Draw rectangle on frame
        detections = []
        # print("="*50)
        for d in data:
            cx = (d[0] + d[0] + d[2])//2
            cy = (d[1] + d[1] + d[3] + d[3])//2
            detections.append([d[4].capitalize(), float(cx),float(cy), d[2]*d[3], 0])
            # print(f'{d[4].capitalize()}: ({cx}, {cy})')
            # print(f'{d[4].capitalize()}: width: {d[2]} height: {d[3]} area: {d[2]*d[3]}')

            # Visualize center points of cones
            cv2.circle(frame, (cx, cy), 5, self.class_colors[d[4]], cv2.FILLED)

            cv2.rectangle(frame, (d[0],d[1]), (d[0]+d[2], d[1]+d[3]), self.class_colors[d[4]], 2)
            cv2.putText(frame, f'{d[4]} {d[5]}%', (d[0],d[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.class_colors[d[4]], 2)
        
        # ---- SORT BEFORE PUBLISH -----
        detections.sort(reverse=True, key=lambda detections: detections[2])

        yellow_array = []
        blue_array = []
        orange_array = []
        result_array = []

        y_id = 1
        b_id = 2
        o_id = 3
        for det in detections:
            if (det[0]=='Yellow'):
                yellow_array.append([float(y_id), det[1], det[2], float(det[3]), float(det[4])])
                result_array.append([float(y_id), det[1], det[2], float(det[3]), float(det[4])])
            elif (det[0]=='Blue'):
                blue_array.append([float(b_id), det[1], det[2], float(det[3]),float(det[4])])
                result_array.append([float(b_id), det[1], det[2], float(det[3]),float(det[4])])
            elif (det[0]=='Orange'):
                orange_array.append([float(o_id), det[1], det[2], float(det[3]),float(det[4])])
                result_array.append([float(o_id), det[1], det[2], float(det[3]),float(det[4])])
        
        # print(detections)
        #print(yellow_array)
        #print(blue_array)

        # ---- ROS PUBLISH ----
        # self.publishCoordinates(yellow_array)
        # self.publishCoordinates(blue_array)
        self.publishCoordinates(result_array)
        """if args.countmsg:
            actSentCones = len(yellow_array) + len(blue_array)
            self.sentCones.append(actSentCones)
            self.countCones += actSentCones """
        
        key = cv2.waitKey(1)

        if key == ord('c'):
            print("Image captured")
            cv2.imwrite("img_old.jpg", frame)

            """if args.countmsg:
            with open(self.scriptDir+"/log/countSentConesDetector.log", 'w') as f:
                f.write(f"Number of Cones sent through Detector: {self.countCones}")
            print(f"Number of Cones sent through Detector: {self.countCones}")
            self.plotting(self.sentCones, "Number of cones sent through Detector") """

        processed_image = cv2.cvtColor(frame, cv2.COLOR_RGB2RGBA)
        processed_image = self.br.cv2_to_imgmsg(frame)
        self.cone_image_publisher.publish(processed_image)

    def maskImage(self, img, output=None):
        blank = np.zeros(img.shape[:2], dtype='uint8')
        pts = np.array([[165, 376], [233, 320],
                        [402, 320], [465, 376],
                        [672, 376], [672, 190],
                        [0, 190], [0, 376]],
                        np.int32)
        pts = pts.reshape((-1,1,2))
        mask = cv2.fillPoly(blank, [pts], (255, 255, 255))
        output = cv2.bitwise_and(img, img, mask=mask)
        return output

    # ***** Find objects of frame *****
    def findObjects(self, outputs, img):
        hT, wT, cT = img.shape
        bbox = []
        classIds = []
        confs = []
        detections = []

        for output in outputs:
            for det in output:
                scores = det[5:]
                classId = np.argmax(scores)
                confidence = scores[classId]
                if confidence > self.confThreshold:
                    w, h = int(det[2]*wT), int(det[3]*hT)
                    x, y = int(det[0]*wT - w/2), int(det[1]*hT - h/2)
                    bbox.append([x,y,w,h])
                    classIds.append(classId)
                    confs.append(float(confidence))
                    detections.append([x, y, w, h])


        # count objects of img
        # print(len(bbox))
        # ----- Plot Number of Detected objects -----
        # if args.plot:
        #     self.number_of_detections.append(len(bbox))

                
        indices = cv2.dnn.NMSBoxes(bbox, confs, self.confThreshold, self.nmsThreshold)
        #print(indices)

        # ----- Save Data of objects (rectangle coordinates, class IDs and confidences) -----
        data = []
        for i in indices:
            try:
                i = i[0]
            except IndexError:
                i = i
            box = bbox[i]
            x,y,w,h = box[0], box[1], box[2], box[3]
            data.append([x, y, w, h, self.classNames[classIds[i]].upper(), int(confs[i]*100)])

        return data
    
    # ***** Plot Detection Results per Frames *****
    def plotting(self, number_of_objects, counted):
        plt.plot(number_of_objects)
        plt.ylabel(counted)
        plt.xlabel('FRAMES')
        plt.show()

    def publishCoordinates(self, detections):
        # Only publish if it does not contain string because of Float32MultiArray
        # for d in detections:
        #     if ('Yellow' in d) or ('Blue' in d) or ('Small_orange' in d):
        #         return
        
        # Convert all data to float
        for data in detections:
            for d in data:
                d = float(d)

        published_data = Float32MultiArray()
        i=1
        for det in detections:
            actual_dim = MultiArrayDimension()
            actual_dim.size = len(det)
            actual_dim.label = "cone_"+str(i)
            published_data.layout.dim.append(actual_dim)
            i+=1
            for d in det:
                published_data.data.append(d)
            # print(f'ROS data:{det}')
        self.cone_coord_publisher.publish(published_data)

def main(args=None):
    rclpy.init(args=args)
    
    scriptDir = os.path.dirname(os.path.realpath(__file__))

    parser = argparse.ArgumentParser()

    parser.add_argument('--weight', type=str, default=scriptDir+'/resources/yolov4-tiny.weights', help='This argument gives the trained network .weight file\'s path for yolo.')
    parser.add_argument('--config', type=str, default=scriptDir+'/resources/yolov4-tiny-custom.cfg', help='This argument gives the necessary file\'s path for yolo.')
    parser.add_argument('--display', action='store_true', help='This argument provides to display the camera/video window to visualize how the detection works.')
    parser.add_argument('--save', help='If given, it saves the detection video as an mp4 formatted file. (You only need to pass the name without .mp4 format)')
    parser.add_argument('--plot', action='store_true', help='This argument provides to plot some measurements in the end of the detection.')
    parser.add_argument('--countmsg', action='store_true', help='This argument provides to count detected cones.') 
    args_detector, unknown = parser.parse_known_args()
    
    detector = ConeDetector(**vars(args_detector))

    rclpy.spin(detector)
    rclpy.shutdown()

if __name__ == '__main__':
    
    main()