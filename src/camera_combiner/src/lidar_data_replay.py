#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from message_filters import ApproximateTimeSynchronizer, Subscriber
from pathlib import Path
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import json


class LiDAR_Data_Replay(Node):
    def __init__(self):
        super().__init__('camera_combiner')

        # Set CVBRIDGE
        self.bridge = CvBridge()
        
        # Read in parameters
        self.declare_parameter('output_dir', '/ros2_object_detection/lidar_data')
        self.output_dir = self.get_parameter('output_dir').value
        self.declare_parameter('category', 'regular_testing_two_people')
        self.category = self.get_parameter('category').value
        self.declare_parameter('output_colormaps', True)
        self.output_colormaps = self.get_parameter('output_colormaps').value
        
        # Define index for counting images
        self.i = 1
        
        # Set the full output directory and subdirectories
        self.output_dir = Path(self.output_dir).joinpath(self.category)
        self.output_dir_lidar = self.output_dir.joinpath('lidar')
        self.output_dir_thermal = self.output_dir.joinpath('thermal')
        self.output_dir_webcam = self.output_dir.joinpath('webcam')
        if self.output_colormaps:
            self.output_dir_lidar_color = self.output_dir.joinpath('lidar_color')
            self.output_dir_thermal_color = self.output_dir.joinpath('thermal_color')
        
        # # Initialize dictionary for max depth
        # self.max_depth_dict = {}
        
        # # Set up timer stuff for outputting dictionary
        # self.timeout_timer = self.create_timer(5.0, self.timer_callback)  # 5 second interval
        # self.last_message_time = self.get_clock().now()
        # self.received_message = False
        
        # Set publishers
        self.lidar_pub = self.create_publisher(Image, '/lidar_output_placeholder', 10)
        self.thermal_pub = self.create_publisher(Image, '/thermal_output_placeholder', 10)
        self.webcam_pub = self.create_publisher(Image, '/webcam_output_placeholder', 10)
        
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
    
    # Timer callback
    def timer_callback(self):
        current_time = self.get_clock().now()
        time_difference = current_time - self.last_message_time
        time_seconds = time_difference.nanoseconds / 1e9
        
        if time_seconds > 5:
            if not self.received_message:
                self.get_logger().info("No message received for 5 seconds, outputting metadata...")
                self.execute_timeout_function()
        else:
            self.received_message = False

    def execute_timeout_function(self):
        # Output max_depth dictionary
        with open(self.output_dir_metadata.joinpath('max_depth.json'), 'w') as json_file:
            json.dump(self.max_depth_dict, json_file)
        
        # End process
        self.get_logger().info("Max depth dictionary written out. Shutting down ROS 2 node...")
        rclpy.shutdown() 
        
    # Callback for writing out synchronized images
    def callback(self, lidar_msg, thermal_msg, webcam_msg): 
        # # Update timer stuff
        # self.received_message = True
        # self.last_message_time = self.get_clock().now()
        
        # Convert image messages to CV2
        try:
            lidar_image = self.bridge.imgmsg_to_cv2(lidar_msg, '32FC1')
            thermal_image = self.bridge.imgmsg_to_cv2(thermal_msg, 'mono16')
            webcam_image = self.bridge.imgmsg_to_cv2(webcam_msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Error while converting image message to CV2: {e}')
        
        # Convert lidar_image to acceptable format
        lidar_image[np.isnan(lidar_image)] = 0
        max_depth = np.max(lidar_image)
        lidar_image_clipped = np.clip(lidar_image, 0, max_depth)
        lidar_image_mm = lidar_image_clipped * 1000
        lidar_image_normalized = cv2.normalize(lidar_image_mm, None, 0, 65535, cv2.NORM_MINMAX)
        lidar_image_8bit = cv2.convertScaleAbs(lidar_image_normalized, alpha=(255.0 / np.max(lidar_image_normalized)))
        lidar_image_equalized = cv2.equalizeHist(lidar_image_8bit)
        
        self.get_logger().info(f'Max depth for image {self.i} is {float(max_depth)}')
        
        # # Add max_depth to dict
        # self.max_depth_dict[f'{self.i}'] = float(max_depth)
        
        # Output lidar images
        filename_lidar = f'lidar_image_{self.i}.tiff'
        cv2.imwrite(self.output_dir_lidar.joinpath(filename_lidar), lidar_image.astype(np.float32))
        
        # Convert thermal_image to acceptable format
        thermal_image_normalized = cv2.normalize(thermal_image, None, 0, 255, cv2.NORM_MINMAX)
        thermal_image_8bit = np.uint8(thermal_image_normalized)
        
        # Output thermal and webcam images
        filename_thermal = f'thermal_image_{self.i}.png'
        cv2.imwrite(self.output_dir_thermal.joinpath(filename_thermal), thermal_image_8bit)
        filename_webcam = f'webcam_image_{self.i}.png'
        cv2.imwrite(self.output_dir_webcam.joinpath(filename_webcam), webcam_image)
        
        # Create colormaps if desired
        if self.output_colormaps:
            lidar_image_colormap = cv2.applyColorMap(lidar_image_equalized, cv2.COLORMAP_JET)
            thermal_image_colormap = cv2.applyColorMap(thermal_image_8bit, cv2.COLORMAP_HOT)
            
            filename_lidar_color = f'lidar_color_image_equalized_{self.i}.png'
            cv2.imwrite(self.output_dir_lidar_color.joinpath(filename_lidar_color), lidar_image_colormap)
            filename_thermal_color = f'thermal_color_image_{self.i}.png'
            cv2.imwrite(self.output_dir_thermal_color.joinpath(filename_thermal_color), thermal_image_colormap)
        
        # Iterate
        self.i += 1
        
        # Publish images for viewing
        self.lidar_pub.publish(self.bridge.cv2_to_imgmsg(lidar_image_equalized))
        self.thermal_pub.publish(self.bridge.cv2_to_imgmsg(thermal_image_8bit))
        self.webcam_pub.publish(self.bridge.cv2_to_imgmsg(webcam_image))


def main():
    rclpy.init()
    node = LiDAR_Data_Replay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()