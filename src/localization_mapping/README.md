# Lokalizasyon ve Haritalama

`bee1_mapping` ROS 2 paketi — EKF sensör füzyonu + slam_toolbox + pointcloud_to_laserscan.

| Bileşen | Açıklama |
|---|---|
| `config/ekf.yaml` | robot_localization EKF (GPS + IMU + odometri füzyonu) |
| `config/slam_toolbox_mapping.yaml` | slam_toolbox harita/konum parametreleri |
| `config/pointcloud_to_laserscan.yaml` | VLP-16 nokta bulutu → LaserScan |
| `launch/mapping.launch.py` | EKF + SLAM + p2l başlatma |
| `launch/save_map.launch.py` | Üretilen haritayı kaydetme |
