# Karar, Rota Planlama ve Kontrol

`path_planning_sonHali` paketi — uçtan uca planlama/karar yığını.

| Modül | Açıklama |
|---|---|
| `decision/` | HFSM karar mekanizması (durumlar, geçişler, görev yöneticisi) |
| `global_planner/` | Hybrid A* (Ackermann kısıtlı) + occupancy costmap |
| `local_planner/` | TEB (Timed-Elastic-Band) yerel yörünge + hız profili |
| `ros2_adapter/` | ROS 2 düğümü + topic dönüşümleri (kontrol köprüsü) |
| `common/`, `io/` | Tipler, geometri, araç parametreleri, GEOJSON yükleyici |
| `tests/` | Birim ve entegrasyon testleri |

Kontrol: üretilen hedef açı/hız, aracın yerleşik PID kontrolcülerine iletilir.
