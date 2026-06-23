# bee1_mapping — Haritalama (SLAM) Paketi

BEE1 otonom araç için `slam_toolbox` tabanlı 2D occupancy grid haritalama.
**ROS 2 Foxy** için, Gazebo simülasyonunda çalışacak şekilde hazırlanmıştır.
KTR Rapor 7.2 (Konumlandırma ve Haritalama) mimarisine uygundur.

## Hat (pipeline)

```
/velodyne_points --[pointcloud_to_laserscan]--> /scan ┐
/odom + /imu/data + /zed2 VO --[EKF]--> odom->base_link ┘--[slam_toolbox]--> /map
                                                                  |
                                                          [map_saver] -> .pgm + .yaml
```

---

## 1. Bağımlılıklar (bir kez)

```bash
sudo apt update
sudo apt install ros-foxy-slam-toolbox ros-foxy-robot-localization \
                 ros-foxy-pointcloud-to-laserscan ros-foxy-nav2-map-server
```

## 2. Paketi kur (build)

Paket klasörünü çalışma alanının `src/` dizinine kopyala:
```
ros2_ws/src/bee1_mapping/
```
Sonra:
```bash
cd ~/ros2_ws
colcon build --packages-select bee1_mapping
source install/setup.bash
```

## 3. Çalıştırma sırası

> Gazebo'yu ayağa kaldıran kişi önce simülasyonu başlatmalı.
> Gazebo şunları yayınlamalı: **/velodyne_points** (PointCloud2), **/odom**,
> **/imu/data**, ve isteğe bağlı **ZED2 görsel odometri**.

**Terminal 1 — (arkadaş) Gazebo** (araç + parkur dünyası).

**Terminal 2 — Haritalama hattını başlat:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch bee1_mapping mapping.launch.py
```

**Terminal 3 — RViz:**
```bash
rviz2
```
Fixed Frame: `map` | Add → Map (`/map`) | Add → LaserScan (`/scan`) | Add → TF

**Terminal 4 — Aracı parkurda gezdir** (teleop). Tüm parkur + alternatif yollar
+ park alanı kapsanmalı. Başlangıca dönünce loop closure ile harita oturur.

## 4. Haritayı kaydet

```bash
ros2 launch bee1_mapping save_map.launch.py map_name:=~/bee1_parkur_haritasi
# veya doğrudan:
ros2 run nav2_map_server map_saver_cli -f ~/bee1_parkur_haritasi
```
Çıktı: `bee1_parkur_haritasi.pgm` + `bee1_parkur_haritasi.yaml`

---

## ⚠️ ENTEGRASYON — Arkadaşının doğrulaması gereken noktalar

**Topic / frame eşleşmeleri** (config dosyalarında):

| Ayar | Dosya | Olması gereken |
|------|-------|----------------|
| Velodyne topic | `mapping.launch.py` (`cloud_in` remap) | Gazebo PointCloud2 topic'i (`/velodyne_points`) |
| Odometri topic | `config/ekf.yaml` (`odom0`) | Gazebo odom topic'i (`/odom`) |
| IMU topic | `config/ekf.yaml` (`imu0`) | `/imu/data` |
| ZED2 VO topic | `config/ekf.yaml` (`odom1`) | ZED2 odom; YOKSA bu bloğu kapat |
| `base_frame` / `target_frame` | slam + p2l config | araç gövde frame'i (`base_link`) |
| `use_sim_time` | tümü | `true` (launch argümanından gelir) |

**TF çakışması (kritik):**
- EKF `odom -> base_link` yayınlar → Gazebo araç eklentisinde **odometri TF
  yayını KAPALI** olmalı.
- `map -> odom`'u **sadece slam_toolbox** yayınlar.

**TF ağacında bulunması gerekenler (arkadaşın URDF'i):**
`base_link` + sensör frame'leri (`velodyne`/`laser`, `imu_link`, ZED2 frame'i).

---

## Hızlı doğrulama

```bash
ros2 topic hz /velodyne_points      # ham lidar akıyor mu
ros2 topic hz /scan                 # dönüşüm çalışıyor mu (p2l)
ros2 topic echo /map --once         # harita üretiliyor mu
ros2 run tf2_tools view_frames      # map->odom->base_link zinciri tam mı
```

| Belirti | Olası sebep |
|---------|-------------|
| `/scan` yok | `cloud_in` remap'i Velodyne topic'iyle uyuşmuyor |
| Harita oluşmuyor | `use_sim_time` false / `/scan` gelmiyor / base_frame uyuşmuyor |
| Harita kayıyor | EKF odom kaynağı (`/odom`) bağlı değil ya da TF çakışıyor |
| EKF veri almıyor | `odom0`/`imu0` topic adları yanlış |

---

## Notlar
- GPS global füzyonu (navsat) ve **AMCL lokalizasyonu** bu pakette YOKTUR;
  onlar harita çıktıktan sonraki **lokalizasyon fazına** aittir (ayrı paket).
- Gazebo'da ZED2 görsel odometri yoksa `ekf.yaml` içindeki `odom1` bloğunu
  kapatın; harita tekerlek odom + IMU ile yine çıkar.
