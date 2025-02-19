from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    # Set bag path for playing
    bag_path = '/ros2_object_detection/images/mannequin_sitting/bag'
    
    # Start model testing node
    model_testing_node = Node(
        package='camera_combiner',
        executable='/ros2_object_detection/src/camera_combiner/src/model_testing.py',
        name='model_testing',
        output='screen',
        remappings=[
            ('/lidar_placeholder', '/flexx2_camera_node/depth_image_rect'),
            ('/thermal_placeholder', '/lepton/image_rect'),
            ('/webcam_placeholder', '/webcam/image_rect'),
            ('/lidar_output_placeholder', 'flexx2_camera_node/detection_depth_image_rect'),
            ('/thermal_output_placeholder', '/lepton/detection_image_rect'),
            ('/webcam_output_placeholder', '/webcam/detection_image_rect')
        ]
    )
    
    # Start RVIZ2 viewing node
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', '/ros2_object_detection/src/camera_combiner/config/model_testing_output.rviz']
    )
    
    # Play the testing bag
    bag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_path],
        output='screen')
    
    return LaunchDescription([model_testing_node, rviz2_node, bag_play])
    # return LaunchDescription([bag_play, rviz2_node])