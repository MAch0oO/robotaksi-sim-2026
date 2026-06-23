# Algılama (Perception)

Çevresel farkındalık modülleri (ROS 2 Foxy).

| Dosya | Görev |
|---|---|
| `lane_follower.py` | ZED2 kamerasından HSV + dilim tabanlı şerit takibi ve PID direksiyon |
| `obstacle_detection.py` | VLP-16 LIDAR Euclidean clustering + takip; statik/dinamik sınıflandırma; kamera füzyonu |
| `yolo_sign_detection.py` | YOLOv8 ile trafik levhası / ışık tespiti (sanal kamera) |

> Not: YOLO model dosyası (`*.pt`) `.gitignore` ile hariç tutulur; ayrı paylaşılır.
