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
    lepton_param_file = join(usb_cam_path, 'config', 'lepton_params_calibrated_no_d`.yaml') #NOTE: change for different parameters if needed
    webcam_param_file = join(usb_cam_path, 'config', 'webcam_params_acircular.yaml') #NOTE: change for different parameters if needed
    
    # LiDAR calibrated camera info publisher
    lidar_info_publisher = Node(
        package='camera_combiner',
        executable='/ros2_object_detection/src/camera_combiner/src/lidar_info_publisher.py',
        name='lidar_info_publisher',
        output='screen',
        remappings=[
            ('/camera_info_placeholder', '/lidar_camera_info_calibrated')
        ],
        parameters=[{
            'yaml_file': '/ros2_object_detection/intrinsic_calibration/lidar/acircular_good/ost.yaml'
        }]
    )

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
    
    # Image Processing node for FLEXX2
    image_proc_lidar_node = Node(
        package='image_proc',
        executable='image_proc',
        name='image_proc_lidar',
        output='screen',
        remappings=[
            ('/image', '/flexx2_camera_node/depth_image_0'),
            ('/camera_info', '/flexx2_camera_node/camera_info'), #NOTE: change for different parameters if needed
            # ('/camera_info', '/lidar_camera_info_calibrated'), #NOTE: change for different parameters if needed
            ('/image_color', '/flexx2_camera_node/depth_image_color'),
            ('/image_color/compressed', '/flexx2_camera_node/depth_image_color/compressed'),
            ('/image_color/compressedDepth', '/flexx2_camera_node/depth_image_color/compressedDepth'),
            ('/image_color/theora', '/flexx2_camera_node/depth_image_color/theora'),
            ('/image_mono', '/flexx2_camera_node/depth_image_mono'),
            ('/image_mono/compressed', '/flexx2_camera_node/depth_image_mono/compressed'),
            ('/image_mono/compressedDepth', '/flexx2_camera_node/depth_image_mono/compressedDepth'),
            ('/image_mono/theora', '/flexx2_camera_node/depth_image_mono/theora'),
            ('/image_rect', '/flexx2_camera_node/depth_image_rect'),
            ('/image_rect/compressed', '/flexx2_camera_node/depth_image_rect/compressed'),
            ('/image_rect/compressedDepth', '/flexx2_camera_node/depth_image_rect/compressedDepth'),
            ('/image_rect/theora', '/flexx2_camera_node/depth_image_rect/theora'),
            ('/image_raw', '/flexx2_camera_node/depth_image_raw_proc')
        ]
    )
    
    # Image Processing node for Thermal
    image_proc_thermal_node = Node(
        package='image_proc',
        executable='image_proc',
        name='image_proc_thermal',
        output='screen',
        remappings=[
            ('/image', '/lepton/image_raw'),
            ('/camera_info', '/lepton/camera_info'),
            ('/image_color', '/lepton/image_color'),
            ('/image_color/compressed', '/lepton/image_color/compressed'),
            ('/image_color/compressedDepth', '/lepton/image_color/compressedDepth'),
            ('/image_color/theora', '/lepton/image_color/theora'),
            ('/image_mono', '/lepton/image_mono'),
            ('/image_mono/compressed', '/lepton/image_mono/compressed'),
            ('/image_mono/compressedDepth', '/lepton/image_mono/compressedDepth'),
            ('/image_mono/theora', '/lepton/image_mono/theora'),
            ('/image_rect', '/lepton/image_rect'),
            ('/image_rect/compressed', '/lepton/image_rect/compressed'),
            ('/image_rect/compressedDepth', '/lepton/image_rect/compressedDepth'),
            ('/image_rect/theora', '/lepton/image_rect/theora'),
            ('/image_raw', '/lepton/image_raw_proc')
        ]
    )
    
    # Image Processing node for Webcam
    image_proc_webcam_node = Node(
        package='image_proc',
        executable='image_proc',
        name='image_proc_webcam',
        output='screen',
        remappings=[
            ('/image', '/webcam/image_raw'),
            ('/camera_info', '/webcam/camera_info'),
            ('/image_color', '/webcam/image_color'),
            ('/image_color/compressed', '/webcam/image_color/compressed'),
            ('/image_color/compressedDepth', '/webcam/image_color/compressedDepth'),
            ('/image_color/theora', '/webcam/image_color/theora'),
            ('/image_mono', '/webcam/image_mono'),
            ('/image_mono/compressed', '/webcam/image_mono/compressed'),
            ('/image_mono/compressedDepth', '/webcam/image_mono/compressedDepth'),
            ('/image_mono/theora', '/webcam/image_mono/theora'),
            ('/image_rect', '/webcam/image_rect'),
            ('/image_rect/compressed', '/webcam/image_rect/compressed'),
            ('/image_rect/compressedDepth', '/webcam/image_rect/compressedDepth'),
            ('/image_rect/theora', '/webcam/image_rect/theora'),
            ('/image_raw', '/webcam/image_raw_proc')
        ]
    )
    
    # Testing Node
    testing_node = Node(
        package='camera_combiner',
        executable='/ros2_object_detection/src/camera_combiner/src/calibration_testing.py',
        name='calibration_testing',
        output='screen',
        remappings=[
            ('/lidar_image_placeholder', '/flexx2_camera_node/depth_image_rect'),
            ('/thermal_image_placeholder', '/lepton/image_rect'),
            ('/webcam_image_placeholder', '/webcam/image_rect'),
            ('/lidar_info_placeholder', '/flexx2_camera_node/camera_info'),
            ('/lidar_calib_info_placeholder', '/lidar_camera_info_calibrated'),
            ('/thermal_info_placeholder', '/lepton/camera_info'),
            ('/webcam_info_placeholder', '/webcam/camera_info'),
            ('/lidar_output', '/flexx2_camera_node/image_target'),
            ('/thermal_output', '/lepton/image_target'),
            ('/webcam_output', '/webcam/image_target')
        ],
        parameters=[{
            'use_lidar_inherent': True, #NOTE: change for different parameters if needed
            'no_extrinsic_rotations': False, #NOTE: change for different parameters if needed
            'pick_center': True, #NOTE: change for different parameters if needed
            'other_point': [112, 86], # u is between 0 & 224, v is between 0 & 172, given in [u, v] #NOTE: change for different parameters if needed
            'cross_color': (255, 255, 255), 
            'cross_thickness': 2,
            'cross_size': 5
        }]
    )
    
    # Testing RVIZ2 Node
    testing_rviz2_node = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', '/ros2_object_detection/src/camera_combiner/config/calibration_testing.rviz']
    )
    
    # Calibration testing launch
    return LaunchDescription([lidar_info_publisher, testing_node, pmd_container, lepton_node, webcam_node, image_proc_lidar_node, image_proc_thermal_node, image_proc_webcam_node, testing_rviz2_node])