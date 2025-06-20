from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()


    cone_detector = Node(
            package='lidar_centerpoint_detector',
            executable='cone_detector',
            parameters=[
            {"/cone_detector/lidar_input_topic": '/points'},
            {"/cone_detector/model_config", '/home/dobayt/git/mmdetection3d/configs/centerpoint/centerpoint_pillar02_second_secfpn_head-dcn_8xb4-cyclic-20e_cone-3d.py'},
            {"/cone_detector/model_checkpoints", '/home/dobayt/ros2_ws/src/formula_student_packages/lidar_centerpoint_detector/lidar_centerpoint_detector/models/ckpt_centerpoint_conescenes/epoch_20_3pts_51_zala_cones.pth'},
            ("/cone_detector/dataset", 'cone'), # only 'cone' or 'nus' are supported
            ],
            name='cone_detector_node',
            output='screen'
        )
    
    ld.add_action(cone_detector)

    return ld
