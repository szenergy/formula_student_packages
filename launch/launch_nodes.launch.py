from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression, EnvironmentVariable
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    conedet_mode = LaunchConfiguration('conedet_mode', default=EnvironmentVariable('CONEDET_MODE', default_value='N')) # N = neural network _lidar, F = lidar + cam, / fusion
    
    # Foxglove bridge node
    foxgl_br = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': 8765,
        }])

    # Cone detection - LiDAR
    ld.add_action(DeclareLaunchArgument("cdl_in_cloud", description="cone_detection_lidar pointcloud topic to process", default_value="cloud_prefiltered"))
    
    # LiDAR pre-filter
    ld.add_action(DeclareLaunchArgument("lpf_in_cloud", description="lidar pre-filter pointcloud topic to process", default_value="nonground"))
    ld.add_action(DeclareLaunchArgument("lpf_in_cones", description="lidar pre-filter marker array topic to process", default_value="yellow_cones"))
    ld.add_action(DeclareLaunchArgument("lpf_out_cloud", description="lidar pre-filter output cloud (pre-filtered)", default_value="cloud_prefiltered"))
    ld.add_action(DeclareLaunchArgument("lpf_out_frame", description="lidar pre-filter output frame (of the pre-filtered/output pcl)", default_value="base_link"))

    # Transform node
    tf = Node(
        package="tf2_ros",
        executable='static_transform_publisher',
        output='screen',
        arguments=[
            "0.466", "0.0", "0.849",
            "0.0", "0.0", "1.0", "0.0",
            "laser_data_frame", "base_link"
        ]
    )
    
    # Kalman filter node
    kalman = Node(
        package="kalman_pos",
        executable='kalman_pos_node',
        output='screen',
        parameters=[
            {"gnss_pose_topic": "gps/nova/current_pose"},
            {"slam_pose_topic": "gps/duro/current_pose"},
            {"vehicle_status_topic": "vehicle_status"},
            {"gnss_covariance_topic": "gps/nova/fix"},
            {"slam_covariance_topic": "gps/duro/fix"},
            {"imu_topic": "imu/data"}, # cog
            {"est_cog_topic": "estimated_pose_cog"},
            {"est_baselink_topic": "estimated_pose_baselink"},
            {"est_accuracy_topic": "estimation_accuracy"},
            {"est_trav_distance_odom_topic": "distance"},
            {"est_trav_distance_est_pos_topic": "estimated_trav_dist_est_pos"},
            {"loop_rate_hz": 20},
            {"gnss_available": False},
            {"slam_available": False},
            {"gnss_accuracy_limit": 10.0},
            {"slam_accuracy_limit": 10.0},
            {"gnss_default_covariance": 15.0},
            {"slam_default_covariance": 15.0},
            {"dynamic_time_calc": True},
            {"do_not_wait_for_gnss_msgs": True},
            {"kinematic_model_max_speed": 0.3},
            {"use_raw_model": False},
            {"orientation_est_enabled": False},
            {"msg_timeout": 2000.0},
            {"invert_yaw_rate": False},
            {"vehicle_param_c1" : 3000.0},
            {"vehicle_param_c2" : 3000.0},
            {"vehicle_param_m" : 180.0},
            {"vehicle_param_jz" : 270.0},
            {"vehicle_param_l1" : 0.624},
            {"vehicle_param_l2" : 0.676},
            {"vehicle_param_swr" : 1.0},
            {"autonomous_mode_topic" : "myrio_state"}
        ]
    )
    
    # Deprojection node (used in fusion only)
    deproj = Node(
        package="deprojection_cpp",
        executable='deprojection',
        output='screen',
        parameters=[
        ]
    )

    # Global mapping node
    gmap = Node(
        package="gmapping",
        executable='gmapping_node',
        output='screen',
        parameters=[
        ]
    )

    # Motion tracking node
    mo_trk = Node(
        package="motion_tracking",
        executable='motion_tracking',
        output='screen',
        parameters=[
        ],
    )


    # Lidar detector / neural network node
    lidar_nnet = Node(
        package='lidar_centerpoint_detector',
        executable='cone_detector',
        parameters=[
            {"/cone_detector/lidar_input_topic": '/points'},
            {"/cone_detector/model_config", '/ros2_ws/src/mmdetection3d/configs/centerpoint/centerpoint_pillar02_second_secfpn_head-dcn_8xb4-cyclic-20e_cone-3d.py'},
            {"/cone_detector/model_checkpoints", '/ros2_ws/src/formula_student_packages/lidar_centerpoint_detector/lidar_centerpoint_detector/models/ckpt_centerpoint_conescenes/epoch_20_3pts_51_zala_cones.pth'},
            ("/cone_detector/dataset", 'cone'), # only 'cone' or 'nus' are supported
            ],
        name='cone_detector_node',
        output='screen',
        # environment=env,
        condition=IfCondition(PythonExpression(["'", conedet_mode, "' == 'N'"]))
    )


    # Lidar pre-filter node (used in fusion only)
    lidar_pre = Node(
        package="lidar_pre_filter",
        executable='filter_vehicle',
        output='screen',
        parameters=[
            {'cloud_in_topic': LaunchConfiguration("lpf_in_cloud")},
            {'cam_cones_topic': LaunchConfiguration("lpf_in_cones")},
            {'output_topic': LaunchConfiguration("lpf_out_cloud")},
            {'output_frame': LaunchConfiguration("lpf_out_frame")},
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
            {'toggle_boundary_trim': True},
            {'toggle_box_filter': True},
            {'toggle_cam_filter': True},

            {'use_sim_time': True}
        ],
        arguments=['--ros-args', '--log-level', 'error'] # tempfix: OLD_TF_DATA error
    )

    # conedet_lidar = Node(
    #     package="cone_detection_lidar",
    #     executable='detection_simple',
    #     output='screen',
    #     parameters=[
    #         {'cloud_in_topic': LaunchConfiguration("cdl_in_cloud")},
    #         {'verbose1': False},
    #         {'verbose2': True},
    #     ]
    # )

    # Lidar detection node (for fusion)
    conedet_lidar = Node(
        package="cone_detection_lidar",
        executable='grid_trajectory',
        output='screen',
        parameters=[
            {'cloud_in_topic': LaunchConfiguration("cdl_in_cloud")},
            {'verbose1': False},
            {'verbose2': True},
            {'visualize_grid': True}, # Just for debug
            {'position_x': -20.0},  # meters
            {'position_y': 0.0},  # meters
            {'cell_size': 0.2},  # meters
            {'length_x': 40.0},  # meters
            {'length_y': 60.0},  # meters
            #{'frame_out': 'os1_sensor'},
            {'mapi_topic_name': 'intensity_grid'},
            {'maph_topic_name': 'height_grid'},
            {'search_length': 4.0}, # meters
            {'search_range_deg': 80.0}, # degrees
            {'search_resolution_deg': 0.5}, # degrees
            {'search_start_mid_deg': -180.0}, # degrees
            {'search_iter': 4}, # positive integer number
        ]
    )

    # Cone detection by camera (for fusion)
    conedet_cam = Node(
        package="detection",
        executable='cone_detection_camera',
        output='screen',
        parameters=[
        ]
    )

    # Fusion node
    fusion = Node(
        package="prcp_object_fusion",
        executable='prcp_object_fusion',
        output='screen',
        parameters=[
        ]
    )

    # Group for starting fusion sub-stack
    fusion_group = GroupAction([
        lidar_pre,
        conedet_lidar,
        deproj,
        conedet_cam,
        fusion
    ], condition=IfCondition(PythonExpression(["'", conedet_mode, "' == 'F'"])) )

    # Planner (FSD) node
    planner = Node(
        package='planner',
        executable='ft_fsd_node',
        parameters=[
        ],
    )
    
    ld.add_action(foxgl_br)
    ld.add_action(tf)
    ld.add_action(kalman)
    ld.add_action(gmap)
    ld.add_action(mo_trk)
    ld.add_action(lidar_nnet)
    ld.add_action(fusion_group)
    ld.add_action(planner)

    return ld
