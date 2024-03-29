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

    tf2_ros_node =  Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id', 'odom', '--child-frame-id', 'base_link'
        ]
    )

    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[('cloud_in', [LaunchConfiguration(variable_name='scanner'), '/refined_map_2D']),
                    ('scan', [LaunchConfiguration(variable_name='scanner'), '/scan'])],
        parameters=[LaunchConfiguration('map_config')],
    )

    orbslam3_run_node = Node(
        package='orbslam3_ros2',
        executable='map',
        name='orbslam3_run',
        arguments=[
            'stereo',
            'src/ORB_SLAM3/Vocabulary/ORBvoc.txt',
            'src/ORB_SLAM3/configuration/D430.yaml',
            'false',
            'true'
        ]
    )

    ld.add_action(orbslam3_run_node)
    ld.add_action(map_processor_node)
    ld.add_action(tf2_ros_node)
    ld.add_action(pointcloud_to_laserscan)

    return ld