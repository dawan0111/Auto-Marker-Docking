from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the YAML file with the parameters
    config = os.path.join(
        get_package_share_directory('auto_marker_docking'),
        'param',
        'docking_param.yaml'
    )

    return LaunchDescription([
        # Define the action server node
        Node(
            package='auto_marker_docking',
            executable='auto_marker_docking_node',
            name='auto_marker_docking_action_server',
            output='screen',
            parameters=[config]
        ),
    ])
