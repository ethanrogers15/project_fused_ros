from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
from os.path import join


def generate_launch_description():
    # Get flexx2 config path
    pmd_royale_ros_examples_path = get_package_share_directory('pmd_royale_ros_examples')
    flexx2_param_file = join(pmd_royale_ros_examples_path, 'config', 'flexx2.yaml')

    # Get lepton & webcam config paths
    usb_cam_path = get_package_share_directory('usb_cam')
    lepton_param_file = join(usb_cam_path, 'config', 'lepton_params.yaml')
    webcam_param_file = join(usb_cam_path, 'config', 'webcam_params.yaml')

    # PMD Royale node container
    pmd_container = ComposableNodeContainer(
        name='pmd_royale_ros_camera_node_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package="tf2_ros",
                plugin='tf2_ros::StaticTransformBroadcasterNode',
                name='static_transform_broadcaster',
                parameters=[{
                    'frame_id' : 'pmd_royale_ros_camera_node_link',
                    'child_frame_id' : 'pmd_royale_ros_camera_node_optical_frame',
                    'translation.x' : 0.0,
                    'translation.y' : 0.0,
                    'translation.z' : 0.0,
                    'rotation.x' : 0.5,
                    'rotation.y': 0.5,
                    'rotation.z' : -0.5,
                    'rotation.w' : -0.5
                }]
            ),
            ComposableNode(
                package='pmd_royale_ros_driver',
                plugin='pmd_royale_ros_driver::CameraNode',
                name='flexx2_camera_node',
                parameters=[flexx2_param_file]
            )
        ],
        output='screen'
    )

    # Lepton Node
    lepton_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='lepton',
        namespace='lepton',
        parameters=[lepton_param_file],
        output='screen'
    )
    
    # Webcam Node
    webcam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='webcam',
        namespace='webcam',
        parameters=[webcam_param_file],
        output='screen'
    )
    
    # Synchronizer node
    synchronizer_node = Node(
        package='camera_combiner',
        executable='/ros2_object_detection/src/camera_combiner/src/resizer.py',
        name='calibration_sync',
        output='screen',
        remappings=[
            ('/lidar_depth_raw_placeholder', '/flexx2_camera_node/depth_image_0'),
            ('/lidar_gray_raw_placeholder', '/flexx2_camera_node/gray_image_0'),
            ('/thermal_raw_placeholder', '/lepton/image_raw'),
            ('/webcam_raw_placeholder', '/webcam/image_raw'),
            ('/lidar_depth_output', '/image/flexx2_camera_node/synchronized_depth_image'),
            ('/lidar_gray_output', '/image/flexx2_camera_node/synchronized_gray_image'),
            ('/thermal_output', '/image/lepton/synchronized_image'),
            ('/webcam_output', '/image/webcam/synchronized_image')
        ]
    )
    
    # Viewing RVIZ2 node
    rviz2_node = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', '/ros2_object_detection/src/camera_combiner/config/calibration_sync.rviz']
    )
    
    # Calibration synchronizer launch
    return LaunchDescription([synchronizer_node, pmd_container, lepton_node, webcam_node, rviz2_node])
    