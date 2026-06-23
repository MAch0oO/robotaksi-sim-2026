import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import message_filters
from cv_bridge import CvBridge
from ultralytics import YOLO
import math
import json

# ==============================================================================
# 🛠️ GAZETBO SİMÜLASYON AYARLARI (TAKIM ARKADAŞINIZDAN ALIP BURAYI DOLDURUN)
# ==============================================================================
# 1. Gazebo'yu kuran arkadaştan 'ros2 topic list' çıktısını isteyip buraları yazın:
RGB_TOPIC = '/x_rgb_topic_buraya'      # Örn: /camera/image_raw
DEPTH_TOPIC = '/y_depth_topic_buraya'  # Örn: /camera/depth/image_raw

# 2. Gazebo'dayken 'ros2 topic echo /camera_info' yapıp K matrisindeki değerleri yazın:
GAZEBO_FX = 527.3  # K matrisindeki 1. eleman [0]
GAZEBO_FY = 527.3  # K matrisindeki 5. eleman [4]
GAZEBO_CX = 640.0  # K matrisindeki 3. eleman [2]
GAZEBO_CY = 360.0  # K matrisindeki 6. eleman [5]

YOLO_MODEL_PATH = 'best.pt' # Colab'dan indirdiğiniz 12 binlik modelin adı
# ==============================================================================

class SignDetector3D(Node):
    def __init__(self):
        super().__init__('sign_detector_3d')
        self.bridge = CvBridge()
        
        # Kamera İç Parametrelerini Sınıfa Tanımla
        self.fx = GAZEBO_FX
        self.fy = GAZEBO_FY
        self.cx = GAZEBO_CX
        self.cy = GAZEBO_CY

        # YOLO Modelini Yükle
        self.get_logger().info("YOLO Modeli Yükleniyor...")
        self.model = YOLO(YOLO_MODEL_PATH)

        # Senkronize Abonelikler (RGB ve Derinlik)
        self.rgb_sub = message_filters.Subscriber(self, Image, RGB_TOPIC)
        self.depth_sub = message_filters.Subscriber(self, Image, DEPTH_TOPIC)
        
        # İki yayını eşleştir (slop: 0.1 saniye tolerans)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.sync_callback)

        # Çıktı Yayıncısı (Algoritma ekibi buradan okuyor)
        self.publisher_ = self.create_publisher(String, '/ai/tabela_3d_konum', 10)
        self.get_logger().info("3B Tabela Tespiti Başarıyla Başlatıldı. Gazebo Verileri Bekleniyor...")

    def sync_callback(self, rgb_msg, depth_msg):
        # ROS mesajlarını OpenCV formatına çevir
        cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        
        # Derinlik formatı genellikle 32-bit float (metre cinsinden) olur
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1") 

        # YOLO ile tespit yap
        results = self.model(cv_image, verbose=False)
        
        for result in results:
            for box in result.boxes:
                class_id = int(box.cls[0])
                class_name = self.model.names[class_id]
                confidence = float(box.conf[0])

                # Güven skoru filtresi (%50 ve üzeri)
                if confidence > 0.50:
                    
                    # Kutunun merkez (u, v) pikselini bul
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    u = int((x1 + x2) / 2)
                    v = int((y1 + y2) / 2)

                    # Görüntü sınırları dışına çıkmayı engelle
                    if u >= depth_image.shape[1] or v >= depth_image.shape[0]:
                        continue

                    # Derinlik haritasından metre cinsinden uzaklığı (Z) çek
                    z_distance = depth_image[v, u]

                    # Eğer mesafe algılanamadıysa (inf veya nan) es geç
                    if not math.isnan(z_distance) and not math.isinf(z_distance) and z_distance > 0:
                        
                        # 2B Pikseli camera_frame cinsinden 3B Koordinata çevir
                        x_3d = (u - self.cx) * z_distance / self.fx
                        y_3d = (v - self.cy) * z_distance / self.fy

                        # Algoritma ekibi için JSON paketini hazırla
                        output_data = {
                            "class_name": class_name,
                            "confidence": round(confidence, 2),
                            "position_3d": {
                                "x": round(x_3d, 2),
                                "y": round(y_3d, 2),
                                "z": round(float(z_distance), 2)
                            }
                        }

                        # Topic üzerinden yayınla (Publish)
                        msg = String()
                        msg.data = json.dumps(output_data)
                        self.publisher_.publish(msg)
                        self.get_logger().info(f"Tabela Bulundu (camera_frame): {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = SignDetector3D()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()