from launch import LaunchDescription
from launch_ros.actions import Node

# ===== KOLAY AYAR (Gazebo'ya gore degistir) =====
MAP_FRAME = 'map'
VEHICLE_FRAME = 'base_footprint'
IMAGE_TOPIC = '/camera/image_raw'   # ZED sol RGB
CMD_TOPIC = '/cmd_vel'                                   # arac komut topic'i
# =================================================


def generate_launch_description():
    return LaunchDescription([
        # map -> odom kopru (izole test icin; tam sistemde EKF/mapping saglar)
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='map_to_odom_static',
             arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']),

        # --- LiDAR destek: engel + guvenlik + lokalizasyon ---
        Node(package='park_perception', executable='roi_filter_node', name='roi_filter_node',
             parameters=[{'x_min': -2.0, 'x_max': 6.0, 'y_min': -2.5, 'y_max': 2.5,
                          'z_min': -0.3, 'z_max': 2.0}]),
        Node(package='park_perception', executable='voxel_grid_node', name='voxel_grid_node',
             parameters=[{'voxel_size': 0.1}]),
        Node(package='park_perception', executable='ransac_ground_node', name='ransac_ground_node',
             parameters=[{'distance_threshold': 0.15, 'ransac_n': 3, 'num_iterations': 1000}]),
        Node(package='park_perception', executable='obstacle_extractor_node', name='obstacle_extractor_node',
             parameters=[{'target_frame': MAP_FRAME, 'cluster_eps': 0.5, 'cluster_min_samples': 5}]),
        Node(package='park_perception', executable='safety_monitor_node', name='safety_monitor_node',
             parameters=[{'corridor_half_width': 1.4, 'z_min': 0.1, 'z_max': 2.0,
                          'threshold_slow': 1.0, 'threshold_stop': 0.4}]),
        Node(package='park_perception', executable='vehicle_pose_publisher', name='vehicle_pose_publisher',
             parameters=[{'target_frame': MAP_FRAME, 'vehicle_frame': VEHICLE_FRAME}]),

        # --- KAMERA park ---
        Node(package='park_perception', executable='camera_park_node', name='camera_park_node',
             parameters=[{'image_topic': IMAGE_TOPIC, 'cmd_topic': CMD_TOPIC,
                          'park_area_ratio': 0.32, 'steer_gain': 1.0}]),
    ])
