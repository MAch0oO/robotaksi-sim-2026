import os
from glob import glob
from setuptools import setup

package_name = 'park_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mert',
    maintainer_email='mert@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_park_node = park_perception.camera_park_node:main',
            'roi_filter_node = park_perception.roi_filter_node:main',
            'voxel_grid_node = park_perception.voxel_grid_node:main',
            'ransac_ground_node = park_perception.ransac_ground_node:main',
            'otsu_line_node = park_perception.otsu_line_node:main',
            'ransac_line_node = park_perception.ransac_line_node:main',
            'kuboid_occupancy_node = park_perception.kuboid_occupancy_node:main',
            'slot_decision_node = park_perception.slot_decision_node:main',
            'vehicle_pose_publisher = park_perception.vehicle_pose_publisher:main',
            'safety_monitor_node = park_perception.safety_monitor_node:main',
            'park_controller_node = park_perception.park_controller_node:main',
            'obstacle_extractor_node = park_perception.obstacle_extractor_node:main',
            'intensity_generator_node = park_perception.intensity_generator_node:main',
        ],
    },
)
