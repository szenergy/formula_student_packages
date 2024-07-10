from launch import LaunchDescription
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument("cloud_topic", description="a pointcloud topic to process", default_value="nonground"),
        DeclareLaunchArgument("cones_topic", description="a marker array topic to process", default_value="yellow_cones"),
        Node(
            package='lidar_pre_filter',
            executable='filter_vehicle',
            output='screen',
            parameters=[
                {'cloud_in_topic': LaunchConfiguration("cloud_topic")},
                {'cam_cones_topic': LaunchConfiguration("cones_topic")},
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
                {'crop_box_array': [    # from base link, [xyz = bwd-left-up]
                    -2.35, -0.25, -0.35, 0.35, -0.25, 0.45, # main body
                    -1.2, -0.5, -0.6, 0.6, -0.25, 0.2,      # side parts
                    -0.25, 0.25, -0.7, 0.7, -0.25, 0.25,    # rear wheels
                    -1.8, -1.2, -0.8, 0.8, -0.25, 0.25,     # front wheels
                    -0.6, -0.15, -0.3, 0.3, 0.4, 1.0,       # seat
                    -1.35, -0.5, -0.25, 0.25, 0.4, 1.0,     # pilot
                ]},
            ]
        )

    ])