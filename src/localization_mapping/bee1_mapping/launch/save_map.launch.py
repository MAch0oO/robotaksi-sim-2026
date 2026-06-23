"""
save_map.launch.py
==================
BEE1 - Haritalama çıktısını (occupancy grid) diske kaydeder.

Harita tamamlandığında (RViz'de parkur tam çıkınca) ayrı bir terminalde
çalıştır. /map topic'ini okuyup .pgm + .yaml olarak yazar.

KULLANIM:
    ros2 launch bee1_mapping save_map.launch.py
    # veya özel isim/dizin:
    ros2 launch bee1_mapping save_map.launch.py map_name:=/home/kullanici/bee1_parkur

Çıktı: <map_name>.pgm  +  <map_name>.yaml   (varsayılan: ~/bee1_parkur_haritasi)

Alternatif (launch yerine doğrudan komut):
    ros2 run nav2_map_server map_saver_cli -f ~/bee1_parkur_haritasi
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    default_map = os.path.join(os.path.expanduser('~'), 'bee1_parkur_haritasi')

    declare_map_name = DeclareLaunchArgument(
        'map_name',
        default_value=default_map,
        description='Kaydedilecek harita yolu (uzantısız). .pgm ve .yaml üretilir.',
    )

    save_map = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
            '-f', LaunchConfiguration('map_name'),
            '--ros-args', '-p', 'save_map_timeout:=10000.0',
        ],
        output='screen',
    )

    return LaunchDescription([
        declare_map_name,
        save_map,
    ])
