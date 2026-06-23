"""ROS2 (Foxy) sarmalayıcı — çekirdek planlama hattını dış dünyaya bağlar.

Bu katman İNCE tutulur: iş mantığı çekirdektedir (``path_planning`` çekirdek modülleri).
Burada yalnızca (a) ROS mesajları ⇄ çekirdek dataclass dönüşümü (``conversions``,
rclpy'siz ve test edilebilir) ve (b) düğüm yaşam döngüsü (``planning_node``, rclpy) yer alır.

Arayüz Sözleşmesi (ekiple onaylandı): ROS2 Foxy. Trajectory tipi
``trajectory_msgs/MultiDOFJointTrajectory`` (Foxy-güvenli). Planlama topic'leri
``/planning/*`` ve ``/perception/*`` namespace'inde; ``/beemobs/*`` araca rezervedir.
"""
