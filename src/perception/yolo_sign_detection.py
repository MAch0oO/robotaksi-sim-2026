import cv2
from ultralytics import YOLO
import logging
from datetime import datetime
import os

# --- 1. LOG YÖNETİMİ YAPILANDIRMASI ---
# Logların kaydedileceği klasör
log_klasoru = "operasyonel_loglar"
if not os.path.exists(log_klasoru):
    os.makedirs(log_klasoru)

# Test için yeni bir dosya adı oluştur
dosya_adi = datetime.now().strftime("log_%Y-%m-%d_%H-%M-%S.txt")
log_yolu = os.path.join(log_klasoru, dosya_adi)

logging.basicConfig(
    filename=log_yolu,
    level=logging.INFO,
    format='%(asctime)s | %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S',
    encoding='utf-8'  # Türkçe karakterler hatasız çıkar
)

logging.info("Kamera başlatıldı")
# --------------------------------------

print("Yapay zeka beyni (best.pt) yükleniyor...")
logging.info("Yapay zeka modeli (best.pt) yukleniyor...")

try:
    model = YOLO('best.pt')
    logging.info("Yapay zeka modeli (best.pt) basariyla yuklendi.")
except Exception as e:
    logging.error(f"HATA: Model yuklenemedi: {e}")
    print(f"HATA: Model yuklenemedi: {e}")
    exit()

# --- KAMERA AYARI ---
# Gazebo'nun oluşturduğu sanal kameranın portunu buraya yazıyoruz.
# Eğer 0 çalışmazsa, bunu 1 veya 2 yaparak sanal kamerayı bulabilirler.
kamera_portu = 0
print(f"Sanal kamera (Port: {kamera_portu}) başlatılıyor...")
logging.info(f"Sanal kamera (Port: {kamera_portu}) baslatiliyor...")
cap = cv2.VideoCapture(kamera_portu)

if not cap.isOpened():
    logging.error("HATA: Kamera bağlantısı kurulamadı!")
    print("HATA: Kamera açılamadı! Gazebo sanal kamerasının çalıştığından emin olun.")
    exit()

# Kamera parametrelerinin okunması ve loglanması
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = cap.get(cv2.CAP_PROP_FPS)
logging.info(f"Kamera Cozunurlugu: {width}x{height} | FPS: {fps} | Baglanti Basarili.")

print("Sistem devrede! Kapatmak için video penceresine tıklayıp 'q' tuşuna bakın.")

while True:
    # Kameradan o anki kareyi oku
    ret, frame = cap.read()
    if not ret:
        logging.warning("Görüntü akışı kesildi!")
        print("Görüntü alınamıyor...")
        break

    try:
        results = model.predict(source=frame, conf=0.75, verbose=False)
    except Exception as e:
        logging.error(f"YOLOv8 Inference Hatasi: {e}")
        print(f"Inference Hatasi: {e}")
        continue

    # TESPİT BİLDİRİLERİNİ LOGLAMA
    for result in results:
        for box in result.boxes:
            cls_id = int(box.cls[0])
            label = model.names[cls_id]
            conf = float(box.conf[0])

            # Log dosyasına kaydetme
            log_mesaji = f"TESPİT: {label:20} | GÜVEN SKORU: {conf:.2f}"
            logging.info(log_mesaji)

    # 3. GÖRSELLEŞTİRME VE Q TUŞU KONTROLÜ
    annotated_frame = results[0].plot()

    # Görüntüyü ekrana bas
    cv2.imshow("Robotaksi - Gazebo Sanal Kamera Testi", annotated_frame)

    # cv2.waitKey(1) -> Videonun akması için 1 milisaniye bekler
    # Klavyeden küçük 'q' tuşuna basılırsa döngüyü kırıp sistemi kapatır
    if cv2.waitKey(1) & 0xFF == ord('q'):
        print("Çıkış yapılıyor...")
        logging.info("Kullanici talebiyle programdan cikiliyor ('q' tusuna basildi).")
        break

# Kapanış işlemleri
cap.release()
cv2.destroyAllWindows()
logging.info("Kamera durduruldu")

