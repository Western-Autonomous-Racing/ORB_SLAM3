from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    map_processor_config = os.path.join(
        get_package_share_directory('map_processor'), 
        'config', 
        'map.yaml')

    map_la = DeclareLaunchArgument(
        'map_config', 
        default_value=map_processor_config,
        description='Descriptions for map ingestion node')

    ld = LaunchDescription([map_la])

    map_processor_node = Node(
        package='map_processor',
        executable='map_processor',
        name='map_processor',
        parameters=[LaunchConfiguration('map_config')]
    )

    tf2_ros_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=[
            '0', '0', '0',
            '0', '0', '0', '1',
            'odom', 'base_link'
        ]
    )

    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[('cloud_in', '/refined_map_2D'),
                    ('scan','/scan')],
        parameters=[{
                'target_frame': 'base_link',
                'transform_tolerance': 0.01,
                'min_height': -0.5,
                'max_height': 0.5,
                'angle_min': -1.5708,  # -M_PI/2
                'angle_max': 1.5708,  # M_PI/2
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.03333,
                'range_min': 0.0,
                'range_max': 100.0,
                'use_inf': False,
                'inf_epsilon': 1.0
            }]
    )

    orbslam3_run_node = Node(
        package='orbslam3_ros2',
        executable='map',
        name='orbslam3_run',
        arguments=[
            'stereo',
            '/home/formatspecifier/Projects/orbslam3_ws/src/ORB_SLAM3/orbslam3_ros2/ORB_SLAM3/Vocabulary/ORBvoc.txt',
            '/home/formatspecifier/Projects/orbslam3_ws/src/ORB_SLAM3/configuration/D430.yaml',
            'false',
            'true'
        ],
    )

    ld.add_action(orbslam3_run_node)
    ld.add_action(map_processor_node)
    ld.add_action(tf2_ros_node)
    ld.add_action(pointcloud_to_laserscan)

    return ld