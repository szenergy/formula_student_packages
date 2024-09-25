from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()


    cone_detector = Node(
            package='lidar_centerpoint_detector',
            executable='cone_detector', # cone_detector_multisweep
            parameters=[
            {"/cone_detector/lidar_input_topic": '/nonground'},
            {"/cone_detector/odom_input_topic": '/nonground'},
            # {"/cone_detector/param2": [10.0, 0.0, 0.0, 1.0]},
            # {"/cone_detector/param3": 'mumps'},
            # {"/cone_detector/param4": True},
            ],
            name='cone_detector_node', # cone_detector_multisweep_node
            output='screen'
        )
    
    ld.add_action(cone_detector)

    return ld
