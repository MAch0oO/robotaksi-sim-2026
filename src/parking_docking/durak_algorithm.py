# NOT: Bu dosya durak manevrası mantığının bir parçasıdır (fragment).
# Bağımsız çalıştırma için ilgili düğüme entegre edilmelidir.
import cv2
import numpy as np

def _durak_manevra(self, frame, width, height):
        # ===== GİRİŞ AYARLARI =====
        KONUM_KARE = 7      # commit sonrası kaç kare DÜZ git (erken kırıp takılıyorsa ARTIR)
        SAG_STEER  = 0.40   # sağa kırma açısı (durağa dalış keskinliği)
        SAG_KARE   = 45     # sağa kaç kare kır (ne kadar uzun süre dalış yapacağı)
        ILERI_KARE = 15     # girince kaç kare düz ilerle (burnu ön duvara çarpıyorsa AZALT)
        SOL_STEER  = 0.30   # çıkışta sola kırma açısı (eski sürüm kalıntısı, artık CIKIS_SOL_STEER kullanılıyor)
        SOL_KARE   = 12     # çıkışta kaç kare sola kır (eski sürüm kalıntısı)
        h, w = frame.shape[:2]
        img_center_x = w / 2.0
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.mavi_alt, self.mavi_ust)
        roi = np.zeros_like(mask)
        roi[int(h * 0.45):h, int(w * 0.35):w] = 255
        mask = cv2.bitwise_and(mask, roi)
        oran = int(np.count_nonzero(mask)) / float(mask.size)
        # CIKIS dışındaysa çıkış kilitlerini temizle
        if self.durak_faz != "CIKIS":
            self._cikis_basladi = False
            self._cikis_tamam = False
        if self.durak_faz == "CIKIS":
            # ===== ÇIKIŞ AYARLARI =====
            CIKIS_DUZ_KARE  = 2     # önce kaç kare DÜZ ilerle (ön duvara tosluyorsa AZALT)
            CIKIS_SOL_KARE  = 85    # sonra kaç kare sola merge (duvara takılırsa ARTIR, karşı şeride uçarsa AZALT)
            CIKIS_SOL_STEER = 0.50  # çıkış sola açısı - SERT (ön duvardan kurtulmak için)
            if getattr(self, '_cikis_tamam', False):
                self._durak_durum_yayinla("YOLDA")
                return False  # çıkış bitti, normal şerit takip
            if not getattr(self, '_cikis_basladi', False):
                self._cikis_basladi = True
                self._cikis_frame = self.frame_counter
            gecen = self.frame_counter - self._cikis_frame
            # Şeridi (sarı+beyaz) net görürse manevrayı erken bırak
            roi_alt = hsv[int(h * 0.6):, :]
            sari  = np.count_nonzero(cv2.inRange(roi_alt, np.array([15, 40, 40]), np.array([36, 255, 255])))
            beyaz = np.count_nonzero(cv2.inRange(roi_alt, np.array([0, 0, 180]), np.array([180, 50, 255])))
            if gecen > CIKIS_DUZ_KARE + 3 and (sari > 50 and beyaz > 50):
                self._cikis_tamam = True
                self._durak_durum_yayinla("YOLDA")
                return False
            # 1. Önce DÜZ ilerle - burnu cepten azıcık öne çıksın
            if gecen < CIKIS_DUZ_KARE:
                self._durak_durum_yayinla("BAYINDE")
                self._yayinla_steer(0.0)
                return True
            # 2. Sonra SERT sola merge - yola gir (duvarı sıyırarak)
            if gecen < CIKIS_DUZ_KARE + CIKIS_SOL_KARE:
                self._durak_durum_yayinla("BAYINDE")
                self._yayinla_steer(+CIKIS_SOL_STEER)
                return True
            # 3. Bitti → şeride bırak (bir daha kırma, daire çizmez)
            self._cikis_tamam = True
            self._durak_durum_yayinla("YOLDA")
            return False
        # ===== BEKLEME =====
        if self.durak_faz == "BEKLEME":
            self._durak_durum_yayinla("BAYINDE")
            self._yayinla_steer(0.0)
            return True
        # ===== GİRİŞ =====
        M = cv2.moments(mask)
        if M["m00"] > 0:
            cx = M["m10"] / M["m00"]
            cy = M["m01"] / M["m00"]
        else:
            cx, cy = img_center_x, 0.0
        cy_norm = cy / float(h)
        if not self._cep_giris_basladi:
            if (oran < self.cep_gorunur_oran
                    or cy_norm < self.cep_yakin_cy
                    or cx < img_center_x):
                self._durak_durum_yayinla("YOLDA")
                return False
            self._cep_giris_basladi = True
            self._cep_giris_frame = self.frame_counter
            self.get_logger().info("[DURAK] Cebe giriş başladı (commit)")
        girilen = self.frame_counter - self._cep_giris_frame
        # 1. KONUMLAN: düz git (bayın yanına gel, erken kırıp takılma)
        if girilen < KONUM_KARE:
            self._durak_durum_yayinla("GIRILIYOR")
            return False
        manevra = girilen - KONUM_KARE
        # 2. SAĞA KIR
        if manevra < SAG_KARE:
            self._durak_durum_yayinla("GIRILIYOR")
            self._yayinla_steer(-SAG_STEER)
            return True
        # 3. AZ ileri (burnu otursun)
        if manevra < SAG_KARE + ILERI_KARE:
            self._durak_durum_yayinla("GIRILIYOR")
            self._yayinla_steer(0.0)
            return True
        # 4. DUR (beyin 30 sn bekler)
        self._durak_durum_yayinla("BAYINDE")
        self._yayinla_steer(0.0)
        return True

