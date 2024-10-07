from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()


    cone_detector = Node(
            package='lidar_centerpoint_detector',
            executable='cone_detector',
            parameters=[
            {"/cone_detector/lidar_input_topic": '/points'},
            {"/cone_detector/model_config", '/home/dobayt/git/mmdetection3d/configs/centerpoint/centerpoint_pillar02_second_secfpn_head-dcn_8xb4-cyclic-20e_nus-3d.py'},
            {"/cone_detector/model_checkpoints", '/home/dobayt/ros2_ws/src/CenterPoint/lidar_centerpoint_detector/lidar_centerpoint_detector/models/ckpt_centerpoint_nuscenes/centerpoint_02pillar_second_secfpn_dcn_4x8_cyclic_20e_nus_20220811_045458-808e69ad.pth'},
            #{"/cone_detector/odom_input_topic": '/odom'},
            # {"/cone_detector/param2": [10.0, 0.0, 0.0, 1.0]},
            # {"/cone_detector/param3": 'mumps'},
            # {"/cone_detector/param4": True},
            ],
            name='cone_detector_node',
            output='screen'
        )
    
    ld.add_action(cone_detector)

    return ld
