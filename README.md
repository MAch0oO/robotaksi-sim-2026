# Robotaksi Simülasyon 2026 — Atlas Otonom

TEKNOFEST 2026 Robotaksi-Binek Otonom Araç Yarışması (Hazır Araç Kategorisi) için geliştirilen otonom sürüş yazılım yığını. Sistem **ROS 2 Foxy** üzerinde çalışır ve **Gazebo** simülasyon ortamında doğrulanır.

## Mimari (Yazılım Katmanları)

| Katman | Dizin | İçerik |
|---|---|---|
| Algılama | `src/perception/` | Şerit takip, engel tanıma, tabela/ışık tanıma (YOLOv8) |
| Lokalizasyon & Haritalama | `src/localization_mapping/` | EKF füzyonu, slam_toolbox, TF |
| Karar, Planlama & Kontrol | `src/decision_planning_control/` | HFSM, Hybrid A* + TEB, kontrol köprüsü |
| Park & Durak | `src/parking_docking/` | Otonom park, yolcu operasyonları, PID besleme |

## Dizin Yapısı

```
robotaksi-sim-2026/
├── src/          # Kaynak kod (modüler)
├── tests/        # Birim ve entegrasyon testleri
├── docs/         # Dokümantasyon
├── .github/      # PR şablonu + CI/CD iş akışları
├── README.md
├── LICENSE
├── CONTRIBUTING.md
└── .gitignore
```

## Kurulum

**Gereksinimler:** Ubuntu 20.04 · ROS 2 Foxy · Python 3.8 · Gazebo Classic 11

```bash
# Depoyu klonla
git clone https://github.com/MAch0oO/robotaksi-sim-2026.git
cd robotaksi-sim-2026

# Python bağımlılıkları
pip install numpy scipy scikit-learn scikit-image opencv-python ultralytics

# ROS 2 paketleri
sudo apt install ros-foxy-robot-localization ros-foxy-slam-toolbox \
                 ros-foxy-pointcloud-to-laserscan
```

## Çalıştırma

```bash
# (ROS 2 çalışma alanı içinde)
colcon build --symlink-install
source install/setup.bash
ros2 launch <paket> bringup.launch.py
```

## Katkı Sağlama

Bu proje **Pull Request** temelli çalışır; `main` ve `develop` dallarına **doğrudan push yasaktır**.
Dallanma standartları, commit kuralları ve inceleme süreci için **[CONTRIBUTING.md](CONTRIBUTING.md)** dosyasını okuyun.

## Lisans

MIT — bkz. [LICENSE](LICENSE).
