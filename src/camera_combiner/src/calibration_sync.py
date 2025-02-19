#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from message_filters import ApproximateTimeSynchronizer, Subscriber
from cv_bridge import CvBridge
import cv2
import numpy as np


class Calibration_Synchronizer(Node):
    def __init__(self):
        super().__init__('camera_combiner')
        
        # Set CVBRIDGE
        self.bridge = CvBridge()
        
        # Set publishers
        self.lidar_depth_publisher = self.create_publisher(Image, '/lidar_depth_output', 10)
        self.lidar_gray_publisher = self.create_publisher(Image, '/lidar_gray_output', 10)
        self.thermal_publisher = self.create_publisher(Image, '/thermal_output', 10)
        self.webcam_publisher = self.create_publisher(Image, '/webcam_output', 10)
        
        # Set subscribers
        self.lidar_depth_sub = Subscriber(self, Image, '/lidar_depth_raw_placeholder')
        self.lidar_gray_sub = Subscriber(self, Image, '/lidar_gray_raw_placeholder')
        self.thermal_sub = Subscriber(self, Image, '/thermal_raw_placeholder')
        self.webcam_sub = Subscriber(self, Image, '/webcam_raw_placeholder')
        
        # Set the synchronizer
        self.sync = ApproximateTimeSynchronizer([self.lidar_depth_sub,
                                                 self.lidar_gray_sub,
                                                 self.thermal_sub, 
                                                 self.webcam_sub],
                                                queue_size=10,
                                                slop=0.1)
        
        # Define the callback
        self.sync.registerCallback(self.callback2)
    
    # Callback for writing out synchronized images
    def callback(self, lidar_depth_msg, lidar_gray_msg, thermal_msg, webcam_msg):
        # Convert image messages to CV2
        try:
            lidar_image = self.bridge.imgmsg_to_cv2(lidar_depth_msg, '32FC1')
            thermal_image = self.bridge.imgmsg_to_cv2(thermal_msg, 'mono16')
        except Exception as e:
            self.get_logger().error(f'Error while converting image messages to CV2: {e}')
        
        # Convert lidar_image to acceptable format
        lidar_image[np.isnan(lidar_image)] = 0
        max_depth = np.max(lidar_image)
        lidar_image_clipped = np.clip(lidar_image, 0, max_depth)
        lidar_image_mm = lidar_image_clipped * 1000
        lidar_image_normalized = cv2.normalize(lidar_image_mm, None, 0, 65535, cv2.NORM_MINMAX)
        lidar_image_8bit = cv2.convertScaleAbs(lidar_image_normalized, alpha=(255.0 / np.max(lidar_image_normalized)))
        lidar_image_equalized = cv2.equalizeHist(lidar_image_8bit)
        
        # Convert thermal_image to acceptable format
        thermal_image_normalized = cv2.normalize(thermal_image, None, 0, 255, cv2.NORM_MINMAX)
        thermal_image_8bit = np.uint8(thermal_image_normalized)
        
        # Convert processed depth image and thermal image back to messages
        modified_lidar_msg = self.bridge.cv2_to_imgmsg(lidar_image_equalized, encoding='passthrough')
        # modified_thermal_msg = self.bridge.cv2_to_imgmsg(thermal_image_8bit, encoding='mono8')
        
        # Convert thermal_image to RGB
        thermal_image_rgb = cv2.applyColorMap(thermal_image_8bit, cv2.COLORMAP_JET)  # Apply a colormap
        modified_thermal_msg = self.bridge.cv2_to_imgmsg(thermal_image_rgb, encoding='rgb8')
        
        # Publish all messages
        self.lidar_depth_publisher.publish(modified_lidar_msg)
        self.lidar_gray_publisher.publish(lidar_gray_msg)
        self.thermal_publisher.publish(modified_thermal_msg)
        self.webcam_publisher.publish(webcam_msg)
    
    def callback2(self, lidar_depth_msg, lidar_gray_msg, thermal_msg, webcam_msg):
        # Convert image messages to CV2
        try:
            lidar_image = self.bridge.imgmsg_to_cv2(lidar_depth_msg, '32FC1')
            thermal_image = self.bridge.imgmsg_to_cv2(thermal_msg, 'mono16')
            webcam_image = self.bridge.imgmsg_to_cv2(webcam_msg, 'bgr8')  # Assuming webcam is in BGR format
        except Exception as e:
            self.get_logger().error(f'Error while converting image messages to CV2: {e}')
            return  # Exit early if conversion fails

        # Define target size
        target_size = (640, 480)  # Set your desired width and height

        # Resize images to target size
        lidar_image_resized = cv2.resize(lidar_image, target_size)
        thermal_image_resized = cv2.resize(thermal_image, target_size)
        webcam_image_resized = cv2.resize(webcam_image, target_size)

        # Process LiDAR image
        lidar_image_resized[np.isnan(lidar_image_resized)] = 0
        max_depth = np.max(lidar_image_resized)
        lidar_image_clipped = np.clip(lidar_image_resized, 0, max_depth)
        lidar_image_mm = lidar_image_clipped * 1000
        lidar_image_normalized = cv2.normalize(lidar_image_mm, None, 0, 65535, cv2.NORM_MINMAX)
        lidar_image_8bit = cv2.convertScaleAbs(lidar_image_normalized, alpha=(255.0 / np.max(lidar_image_normalized)))
        lidar_image_equalized = cv2.equalizeHist(lidar_image_8bit)

        # Process thermal image
        thermal_image_normalized = cv2.normalize(thermal_image_resized, None, 0, 255, cv2.NORM_MINMAX)
        thermal_image_8bit = np.uint8(thermal_image_normalized)

        # Convert processed depth image and thermal image back to messages
        modified_lidar_msg = self.bridge.cv2_to_imgmsg(lidar_image_equalized, encoding='passthrough')
        
        # Convert thermal_image to RGB
        thermal_image_rgb = cv2.applyColorMap(thermal_image_8bit, cv2.COLORMAP_JET)  # Apply a colormap
        modified_thermal_msg = self.bridge.cv2_to_imgmsg(thermal_image_rgb, encoding='rgb8')
        
        # Resize webcam image
        modified_webcam_msg = self.bridge.cv2_to_imgmsg(webcam_image_resized, encoding='bgr8')

        # Publish all messages
        self.lidar_depth_publisher.publish(modified_lidar_msg)
        self.lidar_gray_publisher.publish(lidar_gray_msg)  # Keep as is or process further
        self.thermal_publisher.publish(modified_thermal_msg)
        self.webcam_publisher.publish(modified_webcam_msg)



def main():
    rclpy.init()
    node = Calibration_Synchronizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()