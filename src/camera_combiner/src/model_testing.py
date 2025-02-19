#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from message_filters import ApproximateTimeSynchronizer, Subscriber
from cv_bridge import CvBridge
from pathlib import Path
import cv2
import numpy as np
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision


class Model_Tester(Node):
    def __init__(self):
        super().__init__('camera_combiner')

        # Set CVBRIDGE
        self.bridge = CvBridge()
        
        # Set object detection and visualization parameters
        MAX_RESULTS = 2
        SCORE_THRESHOLD = 0.5 # see everything for AP
        self.MARGIN = 5
        self.ROW_SIZE = -15
        self.FONT_SIZE = 0.5
        self.FONT_THICKNESS = 1
        self.BOX_THICKNESS = 5
        self.TEXT_COLOR = (255, 255, 255)
        
        # Initialize the webcam object detection model
        base_options_webcam = python.BaseOptions(model_asset_path='/ros2_object_detection/models/efficientdet_lite0.tflite')
        # base_options_webcam = python.BaseOptions(model_asset_path='/ros2_object_detection/models/webcam.tflite')
        options_webcam = vision.ObjectDetectorOptions(base_options=base_options_webcam, running_mode=vision.RunningMode.IMAGE, max_results=MAX_RESULTS, score_threshold=SCORE_THRESHOLD)
        self.webcam_detector = vision.ObjectDetector.create_from_options(options_webcam)
        
        # Initialize the thermal object detection model
        base_options_thermal = python.BaseOptions(model_asset_path='/ros2_object_detection/models/thermal.tflite')
        options_thermal = vision.ObjectDetectorOptions(base_options=base_options_thermal, running_mode=vision.RunningMode.IMAGE, max_results=MAX_RESULTS, score_threshold=SCORE_THRESHOLD)
        self.thermal_detector = vision.ObjectDetector.create_from_options(options_thermal)
        
        # Initialize the lidar object detection model
        base_options_lidar = python.BaseOptions(model_asset_path='/ros2_object_detection/models/lidar.tflite')
        options_lidar = vision.ObjectDetectorOptions(base_options=base_options_lidar, running_mode=vision.RunningMode.IMAGE, max_results=MAX_RESULTS, score_threshold=SCORE_THRESHOLD)
        self.lidar_detector = vision.ObjectDetector.create_from_options(options_lidar)
        
        # Set publishers
        self.lidar_pub = self.create_publisher(Image, '/lidar_output_placeholder', 10)
        self.thermal_pub = self.create_publisher(Image, '/thermal_output_placeholder', 10)
        self.webcam_pub = self.create_publisher(Image, '/webcam_output_placeholder', 10)
        
        # Set subscribers
        self.lidar_sub = Subscriber(self, Image, '/lidar_placeholder')
        self.thermal_sub = Subscriber(self, Image, '/thermal_placeholder')
        self.webcam_sub = Subscriber(self, Image, '/webcam_placeholder')
        
        # Define index for counting images
        self.i = 1
        
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
        lidar_image_colormap = cv2.applyColorMap(lidar_image_equalized, cv2.COLORMAP_JET)
        lidar_image_rgb = cv2.cvtColor(lidar_image_equalized, cv2.COLOR_GRAY2RGB)
        # lidar_image_rgb = cv2.cvtColor(lidar_image_8bit, cv2.COLOR_GRAY2RGB)
        # self.get_logger().info(f'LiDAR image number {self.i}')
        # self.get_logger().info(f'Max depth in the {self.i}th image: {max_depth}')
        # self.i += 1
        
        # Convert thermal_image to acceptable format
        thermal_image_normalized = cv2.normalize(thermal_image, None, 0, 255, cv2.NORM_MINMAX)
        thermal_image_8bit = np.uint8(thermal_image_normalized)
        thermal_image_colormap = cv2.applyColorMap(thermal_image_8bit, cv2.COLORMAP_HOT)
        thermal_image_rgb = cv2.cvtColor(thermal_image_8bit, cv2.COLOR_GRAY2RGB)
        
        # Convert webcam_image to acceptable format
        webcam_image_rgb = cv2.cvtColor(webcam_image, cv2.COLOR_BGR2RGB)
        
        # Perform detection 
        lidar_mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=lidar_image_rgb)
        lidar_detection_result = self.lidar_detector.detect(lidar_mp_image)
        thermal_mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=thermal_image_rgb)
        thermal_detection_result = self.thermal_detector.detect(thermal_mp_image)
        webcam_mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=webcam_image_rgb)
        webcam_detection_result = self.webcam_detector.detect(webcam_mp_image)
        
        # View detections
        lidar_detection_image = self.visualize(lidar_image_equalized, lidar_detection_result)
        # lidar_detection_image = self.visualize(lidar_image_8bit, lidar_detection_result)
        thermal_detection_image = self.visualize(thermal_image_8bit, thermal_detection_result)
        webcam_detection_image = self.visualize(webcam_image, webcam_detection_result)
        
        # Publish images with detections
        self.lidar_pub.publish(self.bridge.cv2_to_imgmsg(lidar_detection_image))
        self.thermal_pub.publish(self.bridge.cv2_to_imgmsg(thermal_detection_image))
        self.webcam_pub.publish(self.bridge.cv2_to_imgmsg(webcam_detection_image))
    
    def visualize(self, image, detection_result):
        for detection in detection_result.detections:
            # Draw the bounding box.
            bbox = detection.bounding_box
            start_point = bbox.origin_x, bbox.origin_y
            end_point = bbox.origin_x + bbox.width, bbox.origin_y + bbox.height
            cv2.rectangle(image, start_point, end_point, self.TEXT_COLOR, self.BOX_THICKNESS)

            # Write the label.
            category = detection.categories[0]
            category_name = category.category_name
            probability = round(category.score, 2)
            result_text = category_name + ' (' + str(probability) + ')'
            text_location = (self.MARGIN + bbox.origin_x,
                                self.MARGIN + self.ROW_SIZE + bbox.origin_y)
            cv2.putText(image, result_text, text_location, cv2.FONT_HERSHEY_DUPLEX,
                        self.FONT_SIZE, self.TEXT_COLOR, self.FONT_THICKNESS, cv2.LINE_AA)
            
        return image
            

def main():
    rclpy.init()
    node = Model_Tester()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()