"""
Görevi        : Gerçek aktüatör sürücüsü. Ayrılma ve kanat servolarını
                MAVLink MAV_CMD_DO_SET_SERVO ile sürer. Mock aktüatörlerle
                AYNI arayüzü uygular; üst katmanlar değişmez.
Neden Gerekli : Gereksinim-6 (otonom ayrılma) ve Gereksinim-7 (manuel ayrılma
                komutu). SIMULATION_ONLY'de mock kullanılır; FLIGHT profilinde
                yer istasyonundan gelen AYIR komutunun fiziksel karşılığı olmalı.
İlişkiler     : SeparationSequencer `release()/lock()` çağırır. MavlinkSource'un
                mevcut bağlantısı paylaşılır — ikinci bir MAVLink bağı AÇILMAZ.
                Kanal ve PWM değerleri tools/separation_bench_test.py ile
                sahada doğrulandı.

KANAL HARİTASI (bench aracıyla doğrulandı)
------------------------------------------
    CH14  ayrılma servosu A      LOCKED 1000  ->  RELEASED 2000
    CH13  ayrılma servosu B      LOCKED 1000  ->  RELEASED 2000
    CH15  kanat açma             CLOSED 1000  ->  OPEN     1500

İki ayrılma servosu ZIT yönlü ve EŞZAMANLI sürülür; tek servo arızasında
mekanizma kilitli kalır (bench aracının tasarım notu).

ArduPilot UYARISI
-----------------
DO_SET_SERVO yalnızca SERVOn_FUNCTION = 0 (Disabled) olan kanallarda çalışır.
Kanal bir uçuş fonksiyonuna atanmışsa komut SESSİZCE yok sayılır — servo
kıpırdamaz ve hata da alınmaz. Bench aracı çalıştıysa bu ayar zaten doğrudur;
parametre sıfırlanırsa ilk bakılacak yer burasıdır.

GERİ BİLDİRİM SINIRI — DÜRÜSTLÜK NOTU
--------------------------------------
Servolarda konum geri bildirimi YOKTUR. `released` özelliği "komut gönderildi
ve uçuş kartı kabul etti" anlamına gelir, "ayrılma fiziksel olarak gerçekleşti"
anlamına GELMEZ.

Gerçek doğrulama iki yoldan gelebilir:
  - Ayrılma mekanizmasına limit switch eklemek (en kesin yol)
  - İrtifa profilindeki değişimi izlemek (ayrılma sonrası iniş hızı 12-14'ten
    8-10 m/s'ye düşer)

Bu ayrım ARAS hata kodunu doğrudan etkiler: şartname 2.2, ayrılmanın
gerçekleşmemesi durumunda hata kodu 1 istiyor. Komut gönderildi diye
"gerçekleşti" saymak, gerçekte ayrılmamışsa hakemi yanıltır.
"""
from __future__ import annotations

import time

from src.common.result import ErrorCode, Result


# --- Kanal haritası (bench aracıyla doğrulandı) ---
SEP_A_KANAL = 14
SEP_B_KANAL = 13
WING_KANAL = 15

SEP_LOCKED_PWM = 1000
SEP_RELEASED_PWM = 2000
WING_CLOSED_PWM = 1000
WING_OPEN_PWM = 1500

# Uçuş kartının komutu kabul ettiğini doğrulamak için beklenen süre.
# Ayrılma bir kez olur; bu süre kadar beklemek kabul edilebilir.
ACK_ZAMAN_ASIMI_S = 0.5


class MavlinkServoSurucu:
    """
    MAV_CMD_DO_SET_SERVO ile PWM sürer.

    Mevcut MAVLink bağlantısını PAYLAŞIR; kendi bağlantısını açmaz ve
    kapatmaz. Yaşam döngüsü MavlinkSource'a aittir.
    """

    def __init__(self, connection) -> None:
        self._conn = connection

    @property
    def is_available(self) -> bool:
        return self._conn is not None

    def pwm_yaz(self, kanal: int, pwm: int) -> Result[None]:
        if self._conn is None:
            return Result.err(ErrorCode.UNAVAILABLE, "MAVLink bağlantısı yok")

        try:
            from pymavlink import mavutil
        except ImportError:
            return Result.err(ErrorCode.UNAVAILABLE, "pymavlink kurulu değil")

        try:
            self._conn.mav.command_long_send(
                self._conn.target_system,
                self._conn.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,                      # confirmation
                float(kanal),           # param1: servo numarası
                float(pwm),             # param2: PWM (us)
                0, 0, 0, 0, 0)
        except Exception as exc:
            return Result.err(ErrorCode.IO_ERROR, f"Servo komutu gönderilemedi: {exc}")

        # ACK bekle. Gelmemesi komutun uygulanmadığı anlamına GELMEZ (mesaj
        # kaybolmuş olabilir), ama geldiğinde uçuş kartının kabul ettiğini
        # kesin biliriz.
        bitis = time.monotonic() + ACK_ZAMAN_ASIMI_S
        while time.monotonic() < bitis:
            try:
                msg = self._conn.recv_match(type="COMMAND_ACK", blocking=True,
                                            timeout=0.1)
            except Exception:
                break
            if msg and msg.command == mavutil.mavlink.MAV_CMD_DO_SET_SERVO:
                if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    return Result.ok(None)
                return Result.err(ErrorCode.IO_ERROR,
                                  f"Uçuş kartı komutu reddetti (result={msg.result})")

        # ACK gelmedi ama komut gönderildi — akışı durdurmuyoruz.
        return Result.ok(None)


class RealSeparationServos:
    """
    İki zıt yönlü ayrılma servosu. Mock ile aynı arayüz.

    `released` = "komut gönderildi", "fiziksel olarak ayrıldı" DEĞİL.
    Bkz. modül başındaki geri bildirim notu.
    """

    def __init__(self, surucu: MavlinkServoSurucu,
                 ch_a: int = SEP_A_KANAL, ch_b: int = SEP_B_KANAL) -> None:
        self._surucu = surucu
        self._ch_a = ch_a
        self._ch_b = ch_b
        self._released = False
        self.release_count = 0

    @property
    def released(self) -> bool:
        return self._released

    @property
    def locked(self) -> bool:
        return not self._released

    def release(self) -> Result[None]:
        """
        İki servoyu EŞZAMANLI açar. Biri başarısız olursa hata döner ama
        diğerine komut yine de gönderilmiş olur — yarım açılma, hiç
        açılmamaktan iyidir (mekanizma sıkışabilir ama şansı vardır).
        """
        self.release_count += 1

        a = self._surucu.pwm_yaz(self._ch_a, SEP_RELEASED_PWM)
        b = self._surucu.pwm_yaz(self._ch_b, SEP_RELEASED_PWM)

        if a.is_err:
            return a
        if b.is_err:
            return b

        self._released = True
        return Result.ok(None)

    def to_safe(self) -> Result[None]:
        a = self._surucu.pwm_yaz(self._ch_a, SEP_LOCKED_PWM)
        b = self._surucu.pwm_yaz(self._ch_b, SEP_LOCKED_PWM)

        if a.is_err:
            return a
        if b.is_err:
            return b

        self._released = False
        return Result.ok(None)


class RealWingDeploy:
    """Kanat/kol açma servosu (CH15)."""

    def __init__(self, surucu: MavlinkServoSurucu, kanal: int = WING_KANAL) -> None:
        self._surucu = surucu
        self._kanal = kanal
        self._deployed = False

    @property
    def deployed(self) -> bool:
        return self._deployed

    @property
    def locked(self) -> bool:
        """Preflight kanatlarin guvenli konumda oldugunu denetler."""
        return not self._deployed

    def deploy_and_lock(self) -> Result[None]:
        r = self._surucu.pwm_yaz(self._kanal, WING_OPEN_PWM)
        if r.is_err:
            return r
        self._deployed = True
        return Result.ok(None)

    def to_safe(self) -> Result[None]:
        r = self._surucu.pwm_yaz(self._kanal, WING_CLOSED_PWM)
        if r.is_err:
            return r
        self._deployed = False
        return Result.ok(None)
