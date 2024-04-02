from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true') # if simulation or ros2 bag (mcap) is used, this should be true
    pkg_name = 'speed_control_loop'
    pkg_dir = get_package_share_directory(pkg_name)


    return LaunchDescription([
        Node(
            package='speed_control_loop',
            executable='speed_control',
            parameters=[{
                '/control/P': 250.0,
                '/control/I': 10.0,
                '/control/D': 50.0
                }],
        ),
        Node(
            package='speed_control_loop',
            executable='vehicle_model',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])