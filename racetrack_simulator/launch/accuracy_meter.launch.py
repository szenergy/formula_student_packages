from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='racetrack_simulator',
            executable='accuracy_meter',
            parameters=[
                {"centerline_topic": "racetrack_centerline"}, # topic of the centerline (sub)
                {"predicted_points_topic": "debug_marker"}, # topic of the predicted points (sub)
                {"verbose": False}, # if true, it will print all kinds of debug information
                {"overlay_text_topic": "overlay_text"}, # topic of the overlay text (pub)
                {"accuracy_treshold": 1.0}, # treshold for the accuracy (if the predicted points is farther than this from where it should be, it's a miss)
            ],
        ),

        Node(
            package='rviz_2d_overlay_plugins',
            executable='string_to_overlay_text',
            name='string_to_overlay_text_1',
            output='screen',
            parameters=[
                {"string_topic": "overlay_text"},
                {"fg_color": "b"},  # colors can be: r,g,b,w,k,p,y (red,green,blue,white,black,pink,yellow)
                {"text_size": 100.0}
            ],
        ),
    ])
