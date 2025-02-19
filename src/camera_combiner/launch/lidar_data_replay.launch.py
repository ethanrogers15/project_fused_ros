from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    # Set bag path for playing
    bag_path = '/ros2_object_detection/images/tv/bag' #NOTE: change for different scenarios
    
    # Start lidar replay node
    lidar_data_replay_node = Node(
        package='camera_combiner',
        executable='/ros2_object_detection/src/camera_combiner/src/lidar_data_replay.py',
        name='lidar_data_replay',
        output='screen',
        remappings=[
            ('/lidar_placeholder', '/flexx2_camera_node/depth_image_rect'),
            ('/thermal_placeholder', '/lepton/image_rect'),
            ('/webcam_placeholder', '/webcam/image_rect'),
            ('/lidar_output_placeholder', '/flexx2_camera_node/output'),
            ('/thermal_output_placeholder', '/lepton/output'),
            ('/webcam_output_placeholder', '/webcam/output')
        ],
        parameters=[{
            'output_dir': '/ros2_object_detection/lidar_data',
            'category': 'tv', #NOTE: change for different scenarios
            'output_colormaps': True
        }]
    )
    
    # Start RVIZ2 viewing node
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', '/ros2_object_detection/src/camera_combiner/config/lidar_data_replay_combined.rviz']
    )
    
    # Play the bag
    bag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_path],
        output='screen')
    
    return LaunchDescription([lidar_data_replay_node, rviz2_node, bag_play])
    # return LaunchDescription([bag_play, rviz2_node])