from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    #pkg_name = 'lexus_bringup'
    #pkg_dir = os.popen('/bin/bash -c "source /usr/share/colcon_cd/function/colcon_cd.sh && colcon_cd %s && pwd"' % pkg_name).read().strip()

    #namespace = "lexus3"

    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='gyor0_tf_publisher',
            output='screen',
            arguments=['697237.0', '5285644.0', '0.0','0', '0', '0', '1','map','map_gyor_0'],
            
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='zala0_tf_publisher',
            output='screen',
            arguments=['639770.0', '5195040.0', '0.0','0', '0', '0', '1','map','map_zala_0'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='ouster_base_link_tf_publisher',
            output='screen',
            arguments=['0.466', '0.0', '0.849','0', '0', '0', '1','base_link','laser_sensor_frame'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_gps_tf_publisher',
            output='screen',
            arguments=['-1.88', '0.0', '-0.253', '0.0', '0', '3.14','gps', 'base_link'], 
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='zed_camera_front_tf_publisher',
            output='screen',
            arguments=['0.467', '0.06', '1.011','0', '0.022479841', '0', 'base_link', 'zed2i_left_camera_frame'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_tf_publisher',
            output='screen',
            arguments=['0.564', '0.133', '-0.145','0', '0', '0', '1',  'base_link',  'sensor'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_ground_link_publisher',
            output='screen',
            arguments=['0.0', '0.0', '-0.279','0', '0', '0', '1', 'base_link', 'ground_link'],
        ),
    ])