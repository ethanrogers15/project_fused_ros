#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class Resizer(Node):
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
        self.lidar_depth_sub = self.create_subscription(Image, '/lidar_depth_raw_placeholder', self.lidar_depth_callback, 10)
        self.lidar_gray_sub = self.create_subscription(Image, '/lidar_gray_raw_placeholder', self.lidar_gray_callback, 10)
        self.thermal_sub = self.create_subscription(Image, '/thermal_raw_placeholder', self.thermal_callback, 10)
        self.webcam_sub = self.create_subscription(Image, '/webcam_raw_placeholder', self.webcam_callback, 10)

        # Store the latest images
        self.lidar_depth_image = None
        self.lidar_gray_image = None
        self.thermal_image = None
        self.webcam_image = None

    def lidar_depth_callback(self, msg):
        self.lidar_depth_image = self.bridge.imgmsg_to_cv2(msg, '32FC1')
        self.process_images()

    def lidar_gray_callback(self, msg):
        self.lidar_gray_image = self.bridge.imgmsg_to_cv2(msg, 'mono8')  # Adjust encoding if necessary
        self.process_images()

    def thermal_callback(self, msg):
        self.thermal_image = self.bridge.imgmsg_to_cv2(msg, 'mono16')
        self.process_images()

    def webcam_callback(self, msg):
        self.webcam_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')  # Assuming webcam is in BGR format
        self.process_images()

    def process_images(self):
        # Ensure all images are available
        if (self.lidar_depth_image is not None and 
            self.lidar_gray_image is not None and 
            self.thermal_image is not None and 
            self.webcam_image is not None):
            
            # Define target size
            target_size = (640, 480)  # Set your desired width and height

            # Resize images to target size
            lidar_image_resized = cv2.resize(self.lidar_depth_image, target_size)
            gray_image_resized = cv2.resize(self.lidar_gray_image, target_size)
            thermal_image_resized = cv2.resize(self.thermal_image, target_size)
            webcam_image_resized = cv2.resize(self.webcam_image, target_size)

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
            modified_lidar_msg = self.bridge.cv2_to_imgmsg(lidar_image_equalized, encoding='8UC1')
            # thermal_image_rgb = cv2.applyColorMap(thermal_image_8bit, cv2.COLORMAP_JET)  # Apply a colormap
            modified_thermal_msg = self.bridge.cv2_to_imgmsg(thermal_image_8bit, encoding='mono8')
            modified_webcam_msg = self.bridge.cv2_to_imgmsg(webcam_image_resized, encoding='rgb8')

            # Publish all messages
            self.lidar_depth_publisher.publish(modified_lidar_msg)
            self.lidar_gray_publisher.publish(self.bridge.cv2_to_imgmsg(gray_image_resized, encoding='mono8'))  # Keep as is or process further
            self.thermal_publisher.publish(modified_thermal_msg)
            self.webcam_publisher.publish(modified_webcam_msg)

            # Reset stored images to avoid reprocessing
            self.lidar_depth_image = None
            self.lidar_gray_image = None
            self.thermal_image = None
            self.webcam_image = None


def main():
    rclpy.init()
    node = Resizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
