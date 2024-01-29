#!/usr/bin/env python3
import rclpy

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension

import time

class CoordinatePublisher(object):

    def __init__(self, name):
        self.publisherFloatOnMax = rospy.Publisher(name, Float32MultiArray, queue_size=6)
        self.publisherOnModerate = rospy.Publisher('/cone_moderate', Float32MultiArray, queue_size=1)
        self.subscriber = rospy.Subscriber(name, Float32MultiArray, self.callback,queue_size=1)

    def publishOnModerate(self, msg):
        rate = rospy.Rate(10) #10 Hz
        self.publisherOnModerate.publish(msg)
        rate.sleep()

    def callback(self, msg):
        self.publishOnModerate(msg)

    def publishFloatOnMax(self, detections):

        # Only publish if it does not contain string because of Float32MultiArray
        for d in detections:
            if ('Yellow' in d) or ('Blue' in d):
                return
        
        # Convert all data to float
        for data in detections:
            for d in data:
                d = float(d)
        
        for det in detections:
            published_data = Float32MultiArray(data=det)
            print(f'ROS data:{published_data.data}')
            self.publisherFloatOnMax.publish(published_data)
            time.sleep(0.000001)
