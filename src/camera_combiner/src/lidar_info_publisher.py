#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
import yaml


class CameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('lidar_info_publisher')
        self.publisher_ = self.create_publisher(CameraInfo, '/camera_info_placeholder', 10)
        self.declare_parameter('yaml_file', '/ros2_object_detection/intrinsic_calibration/lidar/acircular_good/ost.yaml')
        self.yaml_file = self.get_parameter('yaml_file').value
        self.camera_info_msg = self.load_camera_info(self.yaml_file)
        self.timer = self.create_timer(0.1, self.publish_camera_info)  # Publish at 10Hz

    def load_camera_info(self, yaml_file):
        # Load YAML data and populate CameraInfo message
        with open(yaml_file, 'r') as file:
            data = yaml.safe_load(file)

        camera_info_msg = CameraInfo()
        camera_info_msg.header.frame_id = data.get('camera_name', 'camera_frame')
        camera_info_msg.height = data['image_height']
        camera_info_msg.width = data['image_width']
        camera_info_msg.distortion_model = data['distortion_model']

        # Accessing the distortion coefficients and other matrices correctly
        camera_info_msg.d = data['distortion_coefficients']['data']
        camera_info_msg.k = data['camera_matrix']['data']
        camera_info_msg.r = data['rectification_matrix']['data']
        camera_info_msg.p = data['projection_matrix']['data']

        return camera_info_msg

    def publish_camera_info(self):
        # Update the timestamp and publish
        self.camera_info_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.camera_info_msg)
        # self.get_logger().info('Published camera_info.')


def main():
    rclpy.init()
    node = CameraInfoPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()