from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true') # if simulation or ros2 bag (mcap) is used, this should be true
    pkg_name = 'planner'
    pkg_dir = get_package_share_directory(pkg_name)


    return LaunchDescription([
        Node(
            package='planner',
            executable='local_planner',
            parameters=[{
                '/planner/P': 250.0
                }],
        ),
    ])