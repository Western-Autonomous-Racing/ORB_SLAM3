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
        parameters=[LaunchConfiguration(map_processor_config)]
    )

    ld.add_action(map_processor_node)

    return ld
