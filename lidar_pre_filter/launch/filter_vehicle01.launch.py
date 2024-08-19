from launch import LaunchDescription
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument("cloud_topic_in", description="name of the pointcloud topic to process", default_value="nonground"),
        DeclareLaunchArgument("cones_topic_in", description="topic of the cone coordinates (input, marker array)", default_value="yellow_cones"),
        DeclareLaunchArgument("out_frame", description="output frame", default_value="base_link"),
        DeclareLaunchArgument("cloud_topic_out", description="name of the output topic (pcl)", default_value="lidar_filter_output"),
        Node(
            package='lidar_pre_filter',
            executable='filter_vehicle',
            output='screen',
            parameters=[
                {'cloud_in_topic': LaunchConfiguration("cloud_topic_in")},
                {'cam_cones_topic': LaunchConfiguration("cones_topic_in")},
                {'output_frame': LaunchConfiguration("out_frame")},
                {'cloud_out_topic': LaunchConfiguration("cloud_topic_out")},
                {'verbose1': False},
                {'verbose2': False},
                {'crop_boundary': [
                #   minX    minY    minZ    maxX    maxY    maxZ
                    -220.0, -220.0, -10.0,  220.0,  220.0,  5.0,
                ]},
                {'crop_box_array': [
                #   minX    minY    minZ    maxX    maxY    maxZ
                    0.25,   -0.35,  -0.25,  2.35,   0.35,   0.45,   # main body
                    0.5,    -0.6,   -0.25,  1.2,    0.6,    0.2,    # side parts
                    -0.25,  -0.7,   -0.25,  0.25,   0.7,    0.25,   # rear wheels
                    1.2,    -0.8,   -0.25,  1.8,    0.8,    0.25,   # front wheels
                    0.15,   -0.3,   0.4,    0.6,    0.3,    1.0,    # seat
                    0.5,    -0.25,  0.4,    1.35,   0.25,   1.0,    # pilot
                ]},
                {'corr_coeffs': [ # dX are the measured differences to be corrected at X distances
                    5,          10,         15,         20,         25,         30,             #X
                    -1.020905,  -1.909484,  -2.275741,  -1.418196,  -0.838905,  0.217884, ]},   #dX
                {'toggle_boundary_trim': True},
                {'toggle_box_filter': True},
                {'toggle_cam_filter': True},
                {'toggle_correct_deproj': False},
            ]
        )

    ])