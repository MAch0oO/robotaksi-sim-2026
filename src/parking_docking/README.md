# Park ve Durak Algoritmaları

| Bileşen | Açıklama |
|---|---|
| `otonom_park/` | Otonom park: `direct_park.py` + `park_perception` ROS 2 paketi (LIDAR slot tespiti, launch'lar) |
| `durak_algorithm.py` | Durak/yolcu operasyonu manevra mantığı |

Park manevra hızı ve yanal kontrol, aracın yerleşik PID kontrolcülerine hedef değer besler.
