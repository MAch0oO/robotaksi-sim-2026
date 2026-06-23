# Otonom Park Algoritması — Kurulum ve Çalıştırma Kılavuzu

TEKNOFEST Robotaksi — Kamera + LiDAR füzyonu ile park cebi algılama ve aracı
**iki park çizgisi arasına park etme**.

Araç şu adımları yapar: park cebini algıla → cebi bir kez kilitle → hizalı sür →
iki çizgi arasında güvenle dur. Test edildi ve çalışıyor (ROS 2 Foxy + Gazebo Classic).

---

## 0) Bu pakette ne var?

```
otonom_park/
├── README.md                         <- bu dosya
├── direct_park.py                    <- PARK SÜRÜŞ kontrolcüsü (ana çalıştırılan dosya)
└── park_perception/                  <- ROS 2 algılama paketi (colcon ile derlenir)
    ├── launch/
    │   ├── park_detect.launch.py     <- BUNU KULLAN (sade, path_planning gerekmez)
    │   └── park_pipeline.launch.py   <- eski/tam sürüm (path_planning ister, KULLANMA)
    └── park_perception/  ... (düğüm kodları)
```

İki parça birlikte çalışır:
- **`park_detect.launch.py`** → algılama zincirini çalıştırır, `/selected_slot_pose`
  (park cebi pozu) ve `/safety_state` üretir.
- **`direct_park.py`** → bu iki konuyu dinler, `/cmd_vel` ile aracı cebe sürüp durdurur.

---

## 1) Ön koşullar (senin bilgisayarında olması gerekenler)

- **ROS 2 Foxy** kurulu ve `source /opt/ros/foxy/setup.bash` çalışıyor.
- **Aynı Gazebo simülasyonu** (bizim kullandığımız robot + park dünyası). Çünkü
  kamera iç parametreleri bu robota göre ayarlı (aşağıda Bölüm 6).
- Python kütüphaneleri: `numpy`, `opencv-python`, `cv_bridge`.
  ```bash
  sudo apt install ros-foxy-cv-bridge python3-opencv
  pip3 install numpy
  ```

### Gazebo simülasyonun ŞU konuları yayınlamalı (en önemli kısım):
| Konu | Tip | Açıklama |
|------|-----|----------|
| `/camera/image_raw` | sensor_msgs/Image | Ön kamera (ZED sol göz) |
| `/lidar/points`     | sensor_msgs/PointCloud2 | LiDAR nokta bulutu |
| `/odom`             | nav_msgs/Odometry | Ackermann sürüş eklentisinden |
| `/cmd_vel`          | geometry_msgs/Twist | Araç buradan sürülür |

TF ağacı: `odom -> base_link -> kamera/lidar frame`leri yayınlanıyor olmalı
(robot_state_publisher). Kameranın optik frame adı `camera_optical_frame` olmalı
(farklıysa `park_detect.launch.py` içindeki `SIGN_FRAME` değerini değiştir).

> Eğer senin Gazebo kameran farklı bir konuya yayın yapıyorsa (örn.
> `/zed_cam/stereo_camera/left/image_raw`), ya o konuyu `/camera/image_raw`a
> remap et, ya da `intensity_generator_node.py` içindeki abonelik konusunu değiştir.

---

## 2) Paketi kur ve derle

```bash
# 1. park_perception klasörünü ros2 workspaceine
