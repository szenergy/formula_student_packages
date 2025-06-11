from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    use_slam_arg = DeclareLaunchArgument(
        'use_slam',
        default_value='false',
        description='Use SLAM for positioning'
    )
    
    use_blue_arg = DeclareLaunchArgument(
        'use_blue_markers',
        default_value='true',
        description='Use blue cone markers'
    )
    
    use_yellow_arg = DeclareLaunchArgument(
        'use_yellow_markers',
        default_value='false',
        description='Use yellow cone markers'
    )
    
    lookahead_arg = DeclareLaunchArgument(
        'lookahead_distance',
        default_value='5.0',
        description='Lookahead distance for path planning'
    )
    
    mission_type_arg = DeclareLaunchArgument(
        'mission_type',
        default_value='trackdrive',
        description='Mission type (trackdrive, skidpad, or acceleration)'
    )

    ft_fsd_node = Node(
        package='planner',
        executable='ft_fsd_node',
        name='ft_fsd',
        parameters=[{
            'use_slam': LaunchConfiguration('use_slam'),
            'use_blue_markers': LaunchConfiguration('use_blue_markers'),
            'use_yellow_markers': LaunchConfiguration('use_yellow_markers'),
            'lookahead_distance': LaunchConfiguration('lookahead_distance'),
            'mission_type': LaunchConfiguration('mission_type'),
        }],
        output='screen'
    )

    return LaunchDescription([
        use_slam_arg,
        use_blue_arg,
        use_yellow_arg,
        lookahead_arg,
        mission_type_arg,
        ft_fsd_node
    ])
