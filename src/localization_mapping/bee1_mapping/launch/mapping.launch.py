"""
mapping.launch.py
=================
BEE1 - KTR Rapor 7.2 uyumlu TAM haritalama hattı / ROS 2 Foxy

Başlattıkları (sırayla):
  1) pointcloud_to_laserscan : /velodyne_points (PointCloud2) -> /scan (LaserScan)
  2) ekf_filter_node         : /odom + /imu/data + ZED2 VO -> odom->base_link TF
  3) async_slam_toolbox_node : /scan + odom -> /map + map->odom

Veri akışı (rapor 7.2):
  /velodyne_points --p2l--> /scan ┐
  /odom + /imu + ZED2 VO --EKF--> odom->base_link ┘--slam_toolbox--> /map

KULLANIM (arkadaş Gazebo'yu ayağa kaldırdıktan SONRA):
    ros2 launch bee1_mapping mapping.launch.py

>>> ENTEGRASYON NOTLARI <<<
- Gazebo /velodyne_points, /odom, /imu/data yayınlamalı.
- EKF 'odom->base_link' TF'ini yayınlar -> Gazebo araç eklentisinde odometri
  TF yayını KAPALI olmalı (yoksa TF çakışır, harita bozulur).
- map->odom'u SADECE slam_toolbox yayınlar (tek kaynak, çakışma yok).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('bee1_mapping')
    p2l_config = os.path.join(pkg_share, 'config', 'pointcloud_to_laserscan.yaml')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')
    slam_config = os.path.join(pkg_share, 'config', 'slam_toolbox_mapping.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Gazebo simülasyon saatini kullan (Gazebo için true).',
    )

    # 1) PointCloud2 -> LaserScan dönüşümü
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan_node',
        output='screen',
        parameters=[p2l_config, {'use_sim_time': use_sim_time}],
        remappings=[
            ('cloud_in', '/velodyne_points'),   # >>> Gazebo Velodyne topic'i
            ('scan', '/scan'),
        ],
    )

    # 2) Sensör füzyonu (yerel EKF)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}],
    )

    # 3) SLAM (slam_toolbox)
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config, {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        declare_use_sim_time,
        pointcloud_to_laserscan_node,
        ekf_node,
        slam_node,
    ])
