import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
from fs_msgs.msg import ControlCommand
from std_msgs.msg import Int8
import numpy as np
import pickle
import os
import pygame


class ModelDriver(Node):
    def __init__(self):
        super().__init__('model_driver')
        
        self.control_pub = self.create_publisher(
            ControlCommand,
            '/fsds/control_command',
            30
        )
        self.yellow_sub = self.create_subscription(
            MarkerArray,
            '/yellow_cones',
            self.yellow_callback,
            10
        )
        self.blue_sub = self.create_subscription(
            MarkerArray,
            '/blue_cones',
            self.blue_callback,
            10
        )
        self.state_sub = self.create_subscription(
            Int8,
            '/fsds/state',
            self.state_callback,
            30
        )
        self.state_value = 0  
        
        self.get_logger().info("Model driver starting (cone-based)...")
        
        self.load_model()
        
        self.safety_distance = 0.2  
        self.max_linear_speed = 1.2  

        self.yellow_cones = []
        self.blue_cones = []

        pygame.init()
        pygame.display.set_mode((100, 100))
        self.w_pressed = False
        self.create_timer(0.05, self.update_keyboard)  

        self.steering_smooth_alpha = 0.2  
        self.prev_steering = 0.0

    def update_keyboard(self):
        pygame.event.pump()
        keys = pygame.key.get_pressed()
        self.w_pressed = keys[pygame.K_w]

    def load_model(self):
        model_path = '/home/szonyibalazs/ros2_ws/src/f1tenth/driving_model_fs.pkl'
        
        if not os.path.exists(model_path):
            self.get_logger().error(f"Model file not found: {model_path}")
            self.get_logger().error("Please run model_trainer.py first!")
            rclpy.shutdown()
            return
        
        try:
            with open(model_path, 'rb') as f:
                self.model_data = pickle.load(f)
                
            self.lin_vel_model = self.model_data['lin_vel_model']
            self.ang_vel_model = self.model_data['ang_vel_model']
            self.expected_features = self.model_data['feature_count']
            
            self.get_logger().info("Model loaded successfully")
            
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")
            rclpy.shutdown()

    def yellow_callback(self, msg):
        self.yellow_cones = [[m.pose.position.x, m.pose.position.y] for m in msg.markers]
        if self.state_value == 3:
            self.predict_and_publish()
        else:
            self.get_logger().info("Waiting for state 3 to predict...")
            
    def blue_callback(self, msg):
        self.blue_cones = [[m.pose.position.x, m.pose.position.y] for m in msg.markers]
        if self.state_value == 3:
            self.predict_and_publish()
        else:
            self.get_logger().info("Waiting for state 3 to predict...")

    def state_callback(self, msg):
        self.state_value = msg.data

    def preprocess_cones(self):
        N = min(3, len(self.yellow_cones), len(self.blue_cones))
        yellow = np.array(self.yellow_cones)[:N]
        blue = np.array(self.blue_cones)[:N]
        midpoints = (yellow + blue) / 2 if yellow.shape == blue.shape else np.zeros((N,2))
        features = np.concatenate([yellow.flatten(), blue.flatten(), midpoints.flatten()])
        
        expected_count = self.expected_features
        if len(features) < expected_count:
            features = np.pad(features, (0, expected_count - len(features)))
        else:
            features = features[:expected_count]
            
        return features.reshape(1, -1)  

    def predict_and_publish(self):
        if not self.yellow_cones or not self.blue_cones:
            return  
        
        try:
            features = self.preprocess_cones()
            
            lin_vel_pred = float(self.lin_vel_model.predict(features)[0])
            ang_vel_pred = float(self.ang_vel_model.predict(features)[0])

            raw_steering = ang_vel_pred * 1.2 
            smoothed_steering = (
                self.steering_smooth_alpha * raw_steering +
                (1 - self.steering_smooth_alpha) * self.prev_steering
            )
            self.prev_steering = smoothed_steering

            min_dist = np.inf
            if self.yellow_cones and self.blue_cones:
                all_cones = np.array(self.yellow_cones + self.blue_cones)
                dists = np.linalg.norm(all_cones, axis=1)
                min_dist = np.min(dists)
            if min_dist < self.safety_distance:
                lin_vel_pred = 0.0
                ang_vel_pred = 0.0
                self.get_logger().warn(f"Emergency stop! Cone at {min_dist:.2f}m")
            
            control_msg = ControlCommand()
            control_msg.throttle = lin_vel_pred * 1.0
            control_msg.steering = smoothed_steering * 1.4
            control_msg.brake = 0.0 if lin_vel_pred > 0.01 else 0.0
            self.control_pub.publish(control_msg)

            if self.count_subscribers('/fsds/control_command') > 0:
                self.get_logger().info(f"Predicted control: throttle={control_msg.throttle:.2f}, steering={control_msg.steering:.2f}, brake={control_msg.brake:.2f}")

        except Exception as e:
            self.get_logger().error(f"Error in prediction: {e}")
            control_msg = ControlCommand()
            control_msg.throttle = 0.0
            control_msg.steering = 0.0
            control_msg.brake = 1.0
            self.control_pub.publish(control_msg)

def main(args=None):
    rclpy.init(args=args)
    driver = ModelDriver()
    
    try:
        rclpy.spin(driver)
    except KeyboardInterrupt:
        driver.get_logger().info("Driver stopped by user")
    finally:
        stop_control = ControlCommand()
        stop_control.throttle = 0.0
        stop_control.steering = 0.0
        stop_control.brake = 1.0
        driver.control_pub.publish(stop_control)
        driver.destroy_node()
        rclpy.shutdown()
        pygame.quit()

if __name__ == '__main__':
    main()
