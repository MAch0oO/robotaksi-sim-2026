from launch import LaunchDescription
from launch_ros.actions import Node

# ===== KOLAY AYAR (Gazebo'ya göre) =====
MAP_FRAME     = 'odom'
VEHICLE_FRAME = 'base_link'
SIGN_FRAME    = 'camera_optical_frame'
CMD_TOPIC     = '/cmd_vel'
N_ESIK        = 50
VOXEL_SIZE    = 0.1
# =======================================


def generate_launch_description():
    return LaunchDescription([
        # Sentetik Yoğunluk Oluşturucu (LIDAR Intensity)
        Node(package='park_perception', executable='intensity_generator_node', name='intensity_generator_node'),

        # ROI Filtresi (Intensity konusunu dinler ve mesafe 12.0m'ye genişletilmiştir)
        Node(package='park_perception', executable='roi_filter_node', name='roi_filter_node',
             parameters=[{'x_min': -2.0, 'x_max': 12.0, 'y_min': -2.5, 'y_max': 2.5,
                          'z_min': -1.2, 'z_max': 2.0}],
             remappings=[('/lidar/points', '/lidar/points_with_intensity')]),

        Node(package='park_perception', executable='voxel_grid_node', name='voxel_grid_node',
             parameters=[{'voxel_size': VOXEL_SIZE}]),

        Node(package='park_perception', executable='ransac_ground_node', name='ransac_ground_node',
             parameters=[{'distance_threshold': 0.15, 'ransac_n': 3, 'num_iterations': 1000}]),

        Node(package='park_perception', executable='otsu_line_node', name='otsu_line_node'),

        Node(package='park_perception', executable='ransac_line_node', name='ransac_line_node',
             parameters=[{'residual_threshold': 0.25}]),

        Node(package='park_perception', executable='kuboid_occupancy_node', name='kuboid_occupancy_node',
             parameters=[{'z_min': 0.15, 'z_max': 2.0, 'n_esik': N_ESIK}]),

        Node(package='park_perception', executable='slot_decision_node', name='slot_decision_node',
             parameters=[{'sign_match_threshold': 3.0, 'confidence_threshold': 0.5,
                           'target_frame': MAP_FRAME, 'sign_frame': SIGN_FRAME}]),

       # Node(package='park_perception', executable='vehicle_pose_publisher', name='vehicle_pose_publisher',
            # parameters=[{'target_frame': MAP_FRAME, 'vehicle_frame': VEHICLE_FRAME}]),

        Node(package='park_perception', executable='safety_monitor_node', name='safety_monitor_node',
             parameters=[{'corridor_half_width': 1.0, 'z_min': 0.1, 'z_max': 2.0,
                           'threshold_slow': 1.0, 'threshold_stop': 0.4}]),

        Node(package='park_perception', executable='obstacle_extractor_node', name='obstacle_extractor_node',
             parameters=[{'target_frame': MAP_FRAME, 'cluster_eps': 0.5, 'cluster_min_samples': 5}]),

        # Yol Planlama Düğümü (Trajectory üreten ana planlayıcı)
        Node(package='path_planning', executable='planning_node', name='path_planning_node',
             remappings=[('/localization/odometry', '/odom')]),

        # Park Kontrolcüsü (Trajectory ve odometriye göre cmd_vel sürüş komutu üretir)
        Node(package='park_perception', executable='park_controller_node', name='park_controller_node',
             parameters=[{'cmd_topic': CMD_TOPIC, 'lookahead': 1.0, 'goal_tolerance': 0.3,
                           'max_speed': 1.0, 'reverse_steer_sign': 1.0}],
             remappings=[('/localization/odometry', '/odom')]),
    ])
