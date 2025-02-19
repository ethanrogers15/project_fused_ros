#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from message_filters import ApproximateTimeSynchronizer, Subscriber
from cv_bridge import CvBridge
from pathlib import Path
import cv2
import numpy as np


class Camera_Combiner(Node):
    def __init__(self):
        super().__init__('camera_combiner')
        
        # Get colormap output choice
        self.declare_parameter('output_colormaps', True)
        self.output_colormaps = self.get_parameter('output_colormaps').value
        self.declare_parameter('category', 'category')
        self.category = self.get_parameter('category').value
        
        if self.output_colormaps:
            self.lidar_color_output_dir = Path(f'/ros2_object_detection/images/{self.category}/lidar_color')
            self.thermal_color_output_dir = Path(f'/ros2_object_detection/images/{self.category}/thermal_color')
        
        # Set image output directory
        self.lidar_output_dir = Path(f'/ros2_object_detection/images/{self.category}/lidar')
        self.thermal_output_dir = Path(f'/ros2_object_detection/images/{self.category}/thermal')
        self.webcam_output_dir = Path(f'/ros2_object_detection/images/{self.category}/webcam')
        
        # Set CVBRIDGE
        self.bridge = CvBridge()
        
        # Set subscribers
        self.lidar_sub = Subscriber(self, Image, '/lidar_placeholder')
        self.thermal_sub = Subscriber(self, Image, '/thermal_placeholder')
        self.webcam_sub = Subscriber(self, Image, '/webcam_placeholder')
        
        # Set the synchronizer
        self.sync = ApproximateTimeSynchronizer([self.lidar_sub, 
                                                 self.thermal_sub, 
                                                 self.webcam_sub],
                                                queue_size=10,
                                                slop=0.15)
        
        # Define the callback
        self.sync.registerCallback(self.callback)
    
    # Callback for writing out synchronized images
    def callback(self, lidar_msg, thermal_msg, webcam_msg):
        # Get timestamps
        lidar_timestamp = lidar_msg.header.stamp.sec + lidar_msg.header.stamp.nanosec * 1E-9
        thermal_timestamp = thermal_msg.header.stamp.sec + thermal_msg.header.stamp.nanosec * 1E-9
        webcam_timestamp = webcam_msg.header.stamp.sec + webcam_msg.header.stamp.nanosec * 1E-9
        avg_timestamp = (lidar_timestamp + thermal_timestamp + webcam_timestamp) / 3
        
        # Convert image messages to CV2
        try:
            lidar_image = self.bridge.imgmsg_to_cv2(lidar_msg, '32FC1')
            thermal_image = self.bridge.imgmsg_to_cv2(thermal_msg, 'mono16')
            webcam_image = self.bridge.imgmsg_to_cv2(webcam_msg, 'bgr8')
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

        # Set image filenames
        lidar_image_file = self.lidar_output_dir.joinpath(f'lidar_image_{avg_timestamp}.png')
        thermal_image_file = self.thermal_output_dir.joinpath(f'thermal_image_{avg_timestamp}.png')
        webcam_image_file = self.webcam_output_dir.joinpath(f'webcam_image_{avg_timestamp}.jpg')
        
        # # Write out image files
        cv2.imwrite(str(lidar_image_file), lidar_image_equalized)
        cv2.imwrite(str(thermal_image_file), thermal_image_8bit)
        cv2.imwrite(str(webcam_image_file), webcam_image)
        
        # Create colormaps if desired
        if self.output_colormaps:
            lidar_image_colormap = cv2.applyColorMap(lidar_image_equalized, cv2.COLORMAP_JET)
            thermal_image_colormap = cv2.applyColorMap(thermal_image_8bit, cv2.COLORMAP_HOT)
            
            lidar_color_image_file = self.lidar_color_output_dir.joinpath(f'lidar_color_image_{avg_timestamp}.png')
            thermal_color_image_file = self.thermal_color_output_dir.joinpath(f'thermal_color_image_{avg_timestamp}.png')
            
            cv2.imwrite(str(lidar_color_image_file), lidar_image_colormap)
            cv2.imwrite(str(thermal_color_image_file), thermal_image_colormap)
            

def main():
    rclpy.init()
    node = Camera_Combiner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()