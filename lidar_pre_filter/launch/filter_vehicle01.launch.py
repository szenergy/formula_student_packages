from launch import LaunchDescription
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument("topic", description="a pointcloud topic to process", default_value="nonground"),
        Node(
            package='lidar_pre_filter',
            executable='filter_vehicle',
            output='screen',
            parameters=[
                {'cloud_in_topic': LaunchConfiguration("topic")},
                {'verbose1': False},
                {'verbose2': False},
                {'minX_over': -220.0},
                {'maxX_over': 220.0},
                {'minY_over': -220.0},
                {'maxY_over': 220.0},
                {'minZ_over': -10.0},
                {'maxZ_over': -0.05},
                {'minX_vehicle': -2.0},
                {'maxX_vehicle': 2.0},
                {'minY_vehicle': -2.0},
                {'maxY_vehicle': 2.0},
            ]
        )

    ])