# ROS2 Adaptör Katmanı (Foxy)

Çekirdek `path_planning` hattının ince ROS2 sarmalayıcısı. İş mantığı çekirdektedir;
burada yalnızca topic ⇄ dataclass dönüşümü (`conversions.py`, rclpy'siz, test edilir)
ve düğüm yaşam döngüsü (`planning_node.py`) vardır.

## Arayüz Sözleşmesi (özet)

**Aboneler (girdiler):**
| Topic | Tip | → Çekirdek |
|---|---|---|
| `/localization/odometry` | `nav_msgs/Odometry` | `vehicle_state` |
| `/perception/obstacles` | `derived_object_msgs/ObjectArray` | `obstacles` (STATIC/DYNAMIC) |
| `/perception/lane_centerline` | `nav_msgs/Path` | `lane.centerline` |
| `/perception/traffic_light_state` (+ `/perception/traffic_light_pose`) | `std_msgs/Int8` (+ `geometry_msgs/PointStamped`) | `traffic_light` |
| `/perception/object_type` (+ `/perception/object_distance`) | `std_msgs/String` (+ `std_msgs/Float32`) | `traffic_sign` (levha etiketi→SignType) |
| `/selected_slot_pose` | `geometry_msgs/PoseStamped` | `park_target` |
| `/safety_state` | `std_msgs/Int8` (0=git,1=yavaşla,2=acil) | `emergency` / hız sınırı |
| `/beemobs/FB_OMUX_to_AUTONOMOUS` | araç paketi (smart_can_*) | `emergency` (OR ile) |

**Yayıncılar (çıktılar):**
| Topic | Tip | ← Çekirdek |
|---|---|---|
| `/planning/trajectory` | `trajectory_msgs/MultiDOFJointTrajectory` | `trajectory` (transform+twist+time) |
| `/planning/trajectory_valid` | `std_msgs/Bool` | `valid` (False→kontrol güvenli dur) |
| `/planning/state` | `std_msgs/String` | HFSM alt durumu |
| `/planning/target_speed` | `std_msgs/Float32` | `target_v` (yavaşla sınırı uygulanmış) |
| `/planning/gear_request` | `std_msgs/Int8` (1=Drive,2=Reverse) | yörünge yön sinyali (park) |
| `/planner/constraint` | `std_msgs/String` (JSON) | levha kısıtları (girilmez, mecburi_yon, park_yasak, tünel…) |
| `/beemobs/rc_unittoOmux` | araç paketi (smart_can_*) | kapı (durak), sinyal, vites, acil — OMUX |

Kurallar: tüm geometri `map` frame'inde; planlama topic'leri asla `/beemobs/*`
kullanmaz (o namespace araca rezerve); `emergency = /safety_state==2 OR FB_EMERGENCY==1`.

## Paketleme (araçta ament_python)
Bu klasör bir ROS2 paketine yerleştirilir. Gereken iki dosya:
- `package.xml` — `<depend>` : `rclpy nav_msgs geometry_msgs std_msgs trajectory_msgs
  builtin_interfaces derived_object_msgs` (+ araç e-stop msg paketi).
- `setup.py` — entry point:
  `console_scripts = ['planning_node = path_planning.ros2_adapter.planning_node:main']`

Derleme/çalıştırma:
```
colcon build --packages-select <paket>
ros2 run <paket> planning_node --ros-args -p geojson_path:=/path/tur.geojson
```

## Açık entegrasyon noktaları (ekiple netleştir)
1. **Araç e-stop mesaj tipi:** `/beemobs/FB_OMUX_to_AUTONOMOUS` için doğru msg paketi/adı
   `_try_subscribe_vehicle_estop` içinde ayarlanmalı (şu an `smart_can_msgs` varsayıldı, yoksa atlanır).
2. **Algı engel tipi:** `derived_object_msgs/ObjectArray` kullanılmıyorsa `conversions.obstacles_*`
   ilgili tipe uyarlanır (duck-typing sayesinde alan eşlemesi yeterli).
3. **Geri vites yön sinyali (yol haritası #4):** TEB şu an pozitif hız üretir; geri park
   segmentlerinin işaretli hızla işaretlenmesi küçük bir çekirdek iyileştirmesidir.
   `gear_request_from_trajectory` o iyileştirme gelince otomatik doğru çalışır.
