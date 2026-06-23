# =====================================================================
# park_detect.launch.py
# Sadelestirilmis PARK ALGILAMA + GUVENLIK pipeline.
# path_planning paketine BAGIMLILIK YOKTUR (planning_node / park_controller
# CIKARILMISTIR). Surus + park manevrasi ayri calisan direct_park.py yapar.
#
# Zincir: kamera+LiDAR fuzyon -> ROI -> voxel -> zemin -> Otsu -> cizgi ->
#         slot_decision -> /selected_slot_pose
#         safety_monitor -> /safety_state
# =====================================================================
from launch import LaunchDescription
from launch_ros.actions import Node

# ----- Gazebo ortaminiza gore ayarlayin -----
MAP_FRAME  = "odom"                  # slot pozunun yayinlanacagi TF frame (direct_park /odom kullanir)
SIGN_FRAME = "camera_optical_frame"  # kameranin optik frame adi
VOXEL_SIZE = 0.1
N_ESIK     = 50
# --------------------------------------------


def generate_launch_description():
    return LaunchDescription([
        # 1) Kamera-LiDAR fuzyonu: beyaz park cizgilerini LiDAR noktalarina intensity=255 olarak isler
        Node(package="park_perception", executable="intensity_generator_node",
             name="intensity_generator_node"),

        # 2) ROI filtre (DIKKAT: z_min=-1.2 yer cizgilerini korur — kritik fix)
        Node(package="park_perception", executable="roi_filter_node", name="roi_filter_node",
             parameters=[{"x_min": -2.0, "x_max": 12.0, "y_min": -2.5, "y_max": 2.5,
                          "z_min": -1.2, "z_max": 2.0}],
             remappings=[("/lidar/points", "/lidar/points_with_intensity")]),

        Node(package="park_perception", executable="voxel_grid_node", name="voxel_grid_node",
             parameters=[{"voxel_size": VOXEL_SIZE}]),

        Node(package="park_perception", executable="ransac_ground_node", name="ransac_ground_node",
             parameters=[{"distance_threshold": 0.15, "ransac_n": 3, "num_iterations": 1000}]),

        Node(package="park_perception", executable="otsu_line_node", name="otsu_line_node"),

        Node(package="park_perception", executable="ransac_line_node", name="ransac_line_node",
             parameters=[{"residual_threshold": 0.25}]),

        Node(package="park_perception", executable="kuboid_occupancy_node", name="kuboid_occupancy_node",
             parameters=[{"z_min": 0.15, "z_max": 2.0, "n_esik": N_ESIK}]),

        # 3) Slot karari -> /selected_slot_pose (MAP_FRAME = odom)
        Node(package="park_perception", executable="slot_decision_node", name="slot_decision_node",
             parameters=[{"sign_match_threshold": 3.0, "confidence_threshold": 0.5,
                          "target_frame": MAP_FRAME, "sign_frame": SIGN_FRAME}]),

        # 4) Guvenlik izleyici -> /safety_state (x_min=0.30: arac govde oz-yansima sahte DUR fixi)
        Node(package="park_perception", executable="safety_monitor_node", name="safety_monitor_node",
             parameters=[{"corridor_half_width": 1.0, "x_min": 0.30, "z_min": 0.1, "z_max": 2.0,
                          "threshold_slow": 1.0, "threshold_stop": 0.4}]),
    ])
