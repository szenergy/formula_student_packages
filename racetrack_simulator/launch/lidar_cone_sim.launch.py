from launch import LaunchDescription
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='racetrack_simulator',
            executable='lidar_cone_sim',
            output='screen',
            parameters=[
                {"track_keypoints": 20}, # number of keypoints when the track is generated
                {"track_radius": 40}, # radius of the track
                {"track_distortion": 0.2}, # distortion factor of the track
                {"num_interpolated_points": 50}, # number of interpolated points between keypoints
                {"cone_spacing": 5.0}, # spacing between cones (on one side)
                {"cone_distance": 1.5}, # distance between cones (the distance between sides and the centerline)
                {"update_frequency": 20}, # update frequency of the simulator
                {"lidar_frame": "laser_data_frame"}, # frame of the simulated lidar
                {"track_pointcloud_topic": "nonground"}, # topic of the track pointcloud (pub)
                {"centerline_topic": "racetrack_centerline"}, # topic of the centerline (pub)
                {"seed": 0}, # when seed is 0, it will be randomly generated
                {"crop_minX": -15.0}, # cropbox
                {"crop_minY": -15.0},
                {"crop_maxX": 15.0},
                {"crop_maxY": 15.0},
                {"visible_pointcloud_topic": "visible_points"}, # topic of the "visible points" (pub) (a point will be published here if it's in the cropbox)
                {"noise_num_points": 400}, # number of noise points around the track
                {"noise_radius": 10}, # radius of the noise points (from the track's points)
            ],
        )
    ])