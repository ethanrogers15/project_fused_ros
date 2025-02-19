#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from message_filters import ApproximateTimeSynchronizer, Subscriber
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from numpy import array
from numpy.linalg import inv


class Testing(Node):
    def __init__(self):
        super().__init__('calibration_testing')
        
        # Get parameters
        self.declare_parameter('use_lidar_inherent', True)
        self.use_lidar_inherent = self.get_parameter('use_lidar_inherent').value
        self.declare_parameter('no_extrinsic_rotations', True)
        self.no_extrinsic_rotations = self.get_parameter('no_extrinsic_rotations').value
        self.declare_parameter('pick_center', True)
        self.pick_center = self.get_parameter('pick_center').value
        self.declare_parameter('other_point', [112, 86])
        self.other_point = self.get_parameter('other_point').value
        
        # Set CVBRIDGE
        self.bridge = CvBridge()
        
        # Set extrinsic translation matrices based on physical measurements, no z translation assumed
        self.T_l2t = array([[1, 0, 0, 0.028],
                             [0, 1, 0, -0.038],
                             [0, 0, 1, 0],
                             [0, 0, 0, 1]])
        self.T_l2w = array([[1, 0, 0, 0.083],
                             [0, 1, 0, -0.035],
                             [0, 0, 1, 0],
                             [0, 0, 0, 1]])
        
        # Set extrinsic rotation matrices from stereo calibration
        if not self.no_extrinsic_rotations:
            self.R_t2cₜ = array([[0.804905, 0.593319, 0.010014],
                            [-0.588094, 0.795337, 0.146920],
                            [0.079206, -0.124146, 0.989098]])
            self.R_l2cₜ = array([[0.813639, 0.571181, 0.108367],
                                [-0.580035, 0.784919, 0.217856],
                                [0.039376, -0.240112, 0.969946]])
            self.R_w2cᵣ = array([[0.903012, -0.397065, -0.164039],
                                [0.397183, 0.917127, -0.033513],
                                [0.163751, -0.034891, 0.985884]])
            self.R_l2cᵣ = array([[0.909488, -0.399788, -0.114025],
                                [0.399705, 0.916314, -0.024592],
                                [0.114314, -0.023211, 0.993173]])
        
        # Set parameters for drawing targets in images
        self.declare_parameter('cross_color', (255, 255, 255))
        self.color = self.get_parameter('cross_color').value
        self.declare_parameter('cross_thickness', 2)
        self.thickness = self.get_parameter('cross_thickness').value
        self.declare_parameter('cross_size', 5)
        self.cross_size = self.get_parameter('cross_size').value
        
        # Set image subscribers
        self.lidar_image_sub = Subscriber(self, Image, '/lidar_image_placeholder')
        self.thermal_image_sub = Subscriber(self, Image, '/thermal_image_placeholder')
        self.webcam_image_sub = Subscriber(self, Image, '/webcam_image_placeholder')
        
        # Set camera info subscribers
        if self.use_lidar_inherent:
            self.Kₗ = array([[205.046875, 0.0, 107.55435943603516],
                             [0.0, 205.046875, 82.43924713134766],
                             [0.0, 0.0, 1.0]])
        else:
            self.Kₗ = array([[207.075164, 0.000000, 107.850982], 
                             [0.000000, 207.089701, 82.904219], 
                             [0.000000, 0.000000, 1.000000]])
        self.thermal_info_sub = Subscriber(self, CameraInfo, '/thermal_info_placeholder')
        self.webcam_info_sub = Subscriber(self, CameraInfo, '/webcam_info_placeholder')
        
        # Set testing publishers
        self.lidar_publisher = self.create_publisher(Image, '/lidar_output', 10)
        self.thermal_publisher = self.create_publisher(Image, '/thermal_output', 10)
        self.webcam_publisher = self.create_publisher(Image, '/webcam_output', 10)
        
        self.get_logger().info('Here')
        
        # Set the synchronizer
        self.sync = ApproximateTimeSynchronizer([self.lidar_image_sub, 
                                                 self.thermal_image_sub, 
                                                 self.webcam_image_sub,
                                                 self.thermal_info_sub,
                                                 self.webcam_info_sub],
                                                queue_size=12,
                                                slop=0.15)
        
        # Define the callback
        if self.no_extrinsic_rotations:
            self.sync.registerCallback(self.callback_no_rotations)
        else:
            self.sync.registerCallback(self.callback_rotations)
        
    def callback_no_rotations(self, lidar_img_msg, thermal_img_msg, webcam_img_msg, thermal_info_msg, webcam_info_msg):
        self.get_logger().info('In callback')
        # Get the intrinsic matrices
        Kₜ = array(thermal_info_msg.k).reshape(3,3)
        Kᵣ = array(webcam_info_msg.k).reshape(3,3)
        
        # Convert image messages to CV2
        try:
            lidar_image = self.bridge.imgmsg_to_cv2(lidar_img_msg, '32FC1')
            thermal_image = self.bridge.imgmsg_to_cv2(thermal_img_msg, 'mono16')
            webcam_image = self.bridge.imgmsg_to_cv2(webcam_img_msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Error while converting image messages to CV2: {e}')
        
        # Copy original LiDAR image to retain depth data
        original_lidar_image = lidar_image.copy()    
            
        # Convert LiDAR and thermal images to acceptable formats for viewing
        lidar_image_colormap, thermal_image_colormap = self.lidar_thermal_proc(lidar_image, thermal_image)
        
        # Get pixel coordinates for LiDAR image
        if self.pick_center:
            uₗ = self.Kₗ[0,2]
            vₗ = self.Kₗ[1,2]
        else:
            uₗ, vₗ = self.other_point[0], self.other_point[1]
        
        # Get depth at center of LiDAR image
        zₗ = original_lidar_image[round(vₗ),round(uₗ)]

        # If depth is not registering (zero), then skip
        if zₗ > 1E-8:
            # Calculate the 3D physical coordinate of the center of the LiDAR image
            pₗ = array([uₗ, vₗ, 1])
            l̂ₗ = inv(self.Kₗ) @ pₗ
            r̄ₗ = zₗ * l̂ₗ
            
            # Perform extrinsic translations to the thermal sensor and webcam
            r̄ₗ = array([r̄ₗ[0],r̄ₗ[1],r̄ₗ[2],1])
            r̄ₜ = self.T_l2t @ r̄ₗ
            r̄ᵣ = self.T_l2w @ r̄ₗ
            r̄ₜ = array([r̄ₜ[0], r̄ₜ[1], r̄ₜ[2]])
            r̄ᵣ = array([r̄ᵣ[0], r̄ᵣ[1], r̄ᵣ[2]])
            
            # Transform 3D coordinate to thermal and webcam pixel coordinates
            r̃ₜ = array([r̄ₜ[0]/r̄ₜ[2], r̄ₜ[1]/r̄ₜ[2], r̄ₜ[2]/r̄ₜ[2]])
            r̃ᵣ = array([r̄ᵣ[0]/r̄ᵣ[2], r̄ᵣ[1]/r̄ᵣ[2], r̄ᵣ[2]/r̄ᵣ[2]])
            pₜ = Kₜ @ r̃ₜ
            pᵣ = Kᵣ @ r̃ᵣ
            uₜ, vₜ = pₜ[0], pₜ[1]
            uᵣ, vᵣ = pᵣ[0], pᵣ[1]
            
            # Draw box on LiDAR, thermal, and webcam images
            lidar_output_image = self.draw_cross(lidar_image_colormap, uₗ, vₗ)
            thermal_output_image = self.draw_cross(thermal_image_colormap, uₜ, vₜ)
            webcam_output_image = self.draw_cross(webcam_image, uᵣ, vᵣ)
            
            # Publish LiDAR, thermal, and webcam images with boxes
            try:
                modified_lidar_msg = self.bridge.cv2_to_imgmsg(lidar_output_image, encoding='passthrough')
                modified_thermal_msg = self.bridge.cv2_to_imgmsg(thermal_output_image, encoding='passthrough')
                modified_webcam_msg = self.bridge.cv2_to_imgmsg(webcam_output_image, encoding='passthrough')
                self.lidar_publisher.publish(modified_lidar_msg)
                self.thermal_publisher.publish(modified_thermal_msg)
                self.webcam_publisher.publish(modified_webcam_msg)
            except CvBridgeError as e:
                self.get_logger().error(f'Error converting back to ROS image: {e}')
                
    def callback_rotations(self, lidar_img_msg, thermal_img_msg, webcam_img_msg, thermal_info_msg, webcam_info_msg):
        # Get the intrinsic matrices
        Kₜ = array(thermal_info_msg.k).reshape(3,3)
        Kᵣ = array(webcam_info_msg.k).reshape(3,3)
        
        # Convert image messages to CV2
        try:
            lidar_image = self.bridge.imgmsg_to_cv2(lidar_img_msg, '32FC1')
            thermal_image = self.bridge.imgmsg_to_cv2(thermal_img_msg, 'mono16')
            webcam_image = self.bridge.imgmsg_to_cv2(webcam_img_msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Error while converting image messages to CV2: {e}')
        
        # Copy original LiDAR image to retain depth data
        original_lidar_image = lidar_image.copy()    
            
        # Convert LiDAR and thermal images to acceptable formats for viewing
        lidar_image_colormap, thermal_image_colormap = self.lidar_thermal_proc(lidar_image, thermal_image)
        
        # Get pixel coordinates for LiDAR image
        if self.pick_center:
            uₗ = self.Kₗ[0,2]
            vₗ = self.Kₗ[1,2]
        else:
            uₗ, vₗ = self.other_point[0], self.other_point[1]
        
        # Get depth at center of LiDAR image
        zₗ = original_lidar_image[round(vₗ),round(uₗ)]

        # If depth is not registering (zero), then skip
        if zₗ > 1E-8:
            # Calculate the 3D physical coordinate of the center of the LiDAR image
            pₗ = array([uₗ, vₗ, 1])
            l̂ₗ = inv(self.Kₗ) @ pₗ
            r̄ₗ = zₗ * l̂ₗ
            
            # Perform extrinsic translations to the thermal sensor and webcam
            r̄ₜ = (inv(self.R_t2cₜ) @ (self.R_l2cₜ @ r̄ₗ)) + array([self.T_l2t[0, 3], self.T_l2t[1, 3], 0]).T
            r̄ᵣ = (inv(self.R_w2cᵣ) @ (self.R_l2cᵣ @ r̄ₗ)) + array([self.T_l2w[0, 3], self.T_l2w[1, 3], 0]).T
            
            # Transform 3D coordinate to thermal and webcam pixel coordinates
            r̃ₜ = array([r̄ₜ[0]/r̄ₜ[2], r̄ₜ[1]/r̄ₜ[2], r̄ₜ[2]/r̄ₜ[2]])
            r̃ᵣ = array([r̄ᵣ[0]/r̄ᵣ[2], r̄ᵣ[1]/r̄ᵣ[2], r̄ᵣ[2]/r̄ᵣ[2]])
            pₜ = Kₜ @ r̃ₜ
            pᵣ = Kᵣ @ r̃ᵣ
            uₜ, vₜ = pₜ[0], pₜ[1]
            uᵣ, vᵣ = pᵣ[0], pᵣ[1]
            
            # Draw box on LiDAR, thermal, and webcam images
            lidar_output_image = self.draw_cross(lidar_image_colormap, uₗ, vₗ)
            thermal_output_image = self.draw_cross(thermal_image_colormap, uₜ, vₜ)
            webcam_output_image = self.draw_cross(webcam_image, uᵣ, vᵣ)
            
            # Publish LiDAR, thermal, and webcam images with boxes
            try:
                modified_lidar_msg = self.bridge.cv2_to_imgmsg(lidar_output_image, encoding='passthrough')
                modified_thermal_msg = self.bridge.cv2_to_imgmsg(thermal_output_image, encoding='passthrough')
                modified_webcam_msg = self.bridge.cv2_to_imgmsg(webcam_output_image, encoding='passthrough')
                self.lidar_publisher.publish(modified_lidar_msg)
                self.thermal_publisher.publish(modified_thermal_msg)
                self.webcam_publisher.publish(modified_webcam_msg)
            except CvBridgeError as e:
                self.get_logger().error(f'Error converting back to ROS image: {e}')
    
    def lidar_thermal_proc(self, lidar_image, thermal_image):
        # LiDAR processing
        lidar_image[np.isnan(lidar_image)] = 0
        max_depth = np.max(lidar_image)
        lidar_image_clipped = np.clip(lidar_image, 0, max_depth)
        lidar_image_mm = lidar_image_clipped * 1000
        lidar_image_normalized = cv2.normalize(lidar_image_mm, None, 0, 65535, cv2.NORM_MINMAX)
        lidar_image_8bit = cv2.convertScaleAbs(lidar_image_normalized, alpha=(255.0 / np.max(lidar_image_normalized)))
        lidar_image_equalized = cv2.equalizeHist(lidar_image_8bit)
        lidar_image_colormap = cv2.applyColorMap(lidar_image_equalized, cv2.COLORMAP_JET)
        
        # Thermal processing
        min_val, max_val, _, _ = cv2.minMaxLoc(thermal_image)
        self.get_logger().info(f'Max: {max_val}, Min: {min_val}')
        thermal_image_normalized = cv2.normalize(thermal_image, None, 0, 255, cv2.NORM_MINMAX)
        thermal_image_8bit = np.uint8(thermal_image_normalized)
        mask = thermal_image_8bit > 0
        masked_image = thermal_image_8bit * mask.astype(np.uint8)
        if np.any(mask):  # Ensure there are non-zero pixels
            equalized_image = cv2.equalizeHist(masked_image)
        else:
            equalized_image = masked_image
            
        # min_val, max_val, _, __ = cv2.minMaxLoc(thermal_image)
        # self.get_logger().info(f'Max: {max_val}, Min: {min_val}')
        # thermal_image_normalized = cv2.normalize(thermal_image, None, 0, 255, cv2.NORM_MINMAX)
        # thermal_image_8bit = np.uint8(thermal_image_normalized)
        # new_min_val, new_max_val, ___, _____ = cv2.minMaxLoc(thermal_image_8bit)
        # self.get_logger().info(f'New Max: {new_max_val}, New Min: {new_min_val}')
        # self.get_logger().info(f'Image: {thermal_image_8bit}')
        # thermal_image_colormap = cv2.applyColorMap(thermal_image_8bit, cv2.COLORMAP_HOT)
        
        return lidar_image_colormap, equalized_image #thermal_image_colormap
    
    def draw_cross(self, image, u, v):
        # Draw the two diagonal lines to form the "X"
        cv2.line(image, (int(u - self.cross_size), int(v - self.cross_size)), 
            (int(u + self.cross_size), int(v + self.cross_size)), self.color, self.thickness)

        cv2.line(image, (int(u - self.cross_size), int(v + self.cross_size)), 
            (int(u + self.cross_size), int(v - self.cross_size)), self.color, self.thickness)

        return image


def main():
    rclpy.init()
    node = Testing()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()