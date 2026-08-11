"""
Görevi        : FLIGHT/HIL profilinde AYRILMA ve KANAT servolarını GERÇEK PCA9685
                (I²C PWM) üzerinden süren aktüatör suite'i. Mock ile BİREBİR aynı
                arayüzü uygular (release/released/locked/to_safe, deploy_and_lock);
                yalnız `separation` ve `arms` fiziksel PWM üretir. Motor/APAM/buzzer
                mock kalır (onlar MAVLink/ApamActuator üzerinden ayrı sürülür).
Neden Gerekli : Yer istasyonundan 'AYIR' (Manuel Ayrılma) komutu ve otonom ayrılma,
                ana döngüde `actuators.separation`'ı sürer. SIMULATION'da bu mock'tur;
                FLIGHT'ta gerçek servolar dönmeliydi ama suite HER ZAMAN mock'tu. Bu
                sürücü boşluğu kapatır: Gereksinim-7 (manuel ayrılma) fiziksel gerçekleşir.
İlişkiler     : SeparationSequencer.release()/released ve ArmDeploySequencer.
                deploy_and_lock()/deployed/locked bu sınıfları sürer (mock ile aynı
                sözleşme). Kanal/PWM değerleri tools/separation_bench_test.py'de
                fiziksel olarak KALİBRE edildi; buradaki sabitler onunla aynı olmalı.
Nasıl Test    : tests/test_real_actuators.py — donanımsız (bus=None) mantık: release
                sonrası released/pozisyon, to_safe LOCKED, kanat deploy. Fiziksel
                doğrulama RPi'de tools/separation_bench_test.py --separate ile yapılır.
DÜRÜSTLÜK NOTU: Gerçek mikroswitch/limit-switch DONANIMDA YOK. `released` ve kol
                `locked` geri bildirimi bu yüzden KOMUT-TABANLIDIR (komut = onay),
                tıpkı mock'taki gibi. smbus2/PCA9685 yoksa SESSİZCE ÇÖKMEZ; açık hata
                tutar ve set_us no-op olur (real_lora deseni).
"""
from __future__ import annotations

import time

from src.common.result import Result
from src.drivers.mock_actuators import (
    ActuatorSuite,
    MockArmMechanism,
    MockSeparationMechanism,
    MockServo,
)
from src.hal.interfaces import ServoPosition

# ── Kalibre değerler — tools/separation_bench_test.py ile AYNI olmalı ──────────
# CH14/CH13 ayrılma (zıt yön), CH15 kanat. µs değerleri fiziksel ölçümle bulundu.
CH_SEP_LEFT = 14       # ayrılma servosu:        LOCKED 1000 → OPEN 1650
CH_SEP_RIGHT = 13      # ayrılma servosu (zıt):  LOCKED 1650 → OPEN 1000
CH_WINGS = 15          # kanat açma servosu:     LOCKED 1000 → OPEN 1500
# APAM paraşüt servosu (SG90) da AYNI PCA9685'te CH12'de. µs değerleri tezgahta
# fiziksel KALİBRE EDİLDİ (2026-08-11): kapalı/başlangıç YÜKSEK, açık/bırak DÜŞÜK.
CH_APAM = 12           # paraşüt servosu:        CLOSED 2100 → OPEN 1000 (kalibre)

_SEP_LEFT_US = {"locked": 1000, "open": 1650}
_SEP_RIGHT_US = {"locked": 1650, "open": 1000}
_WINGS_US = {"locked": 1000, "open": 1500}
_APAM_US = {"closed": 2100, "open": 1000}   # kalibre (apam_bench_test ile aynı)

# PCA9685 I²C
_I2C_BUS = 1
_PCA9685_ADDR = 0x40
_PWM_FREQ_HZ = 50.0
_PERIOD_US = 1_000_000.0 / _PWM_FREQ_HZ

# PCA9685 register haritası
_MODE1 = 0x00
_PRESCALE = 0xFE
_LED0_ON_L = 0x06


def _us_to_counts(pulse_us: float) -> int:
    """Darbe genişliğini (µs) 12-bit PCA9685 sayımına (0..4095) çevirir."""
    counts = int(round(4096.0 * pulse_us / _PERIOD_US))
    return max(0, min(4095, counts))


class Pca9685:
    """
    Minimal PCA9685 servo sürücüsü (smbus2). tools/separation_bench_test.py'deki
    KANITLANMIŞ init/set_us mantığının FSW içi eşdeğeri (tek gövdede kalması için
    kopyalandı — tezgah testi bağımsız/kanıtlı kalsın). `open()` çağrılana dek bağlı
    değildir; smbus2/bus yoksa ÇÖKMEZ, açık hata metni tutar ve set_us no-op olur.
    """

    def __init__(self, log=print) -> None:
        self._log = log
        self._bus = None
        self.error: str | None = None

    @property
    def is_open(self) -> bool:
        return self._bus is not None

    def open(self) -> bool:
        try:
            from smbus2 import SMBus  # type: ignore  # yalnız RPi/donanım profilinde
        except ImportError:
            self.error = "smbus2 kurulu değil (pip install -r requirements-hardware.txt)"
            return False
        try:
            self._bus = SMBus(_I2C_BUS)
            self._init_chip()
        except Exception as exc:  # I²C/bus hataları — açık raporla, çökme yok
            self.error = f"PCA9685 açılamadı (I2C-{_I2C_BUS} @0x{_PCA9685_ADDR:02X}): {exc}"
            self._bus = None
            return False
        return True

    def _init_chip(self) -> None:
        prescale = int(round(25_000_000.0 / (4096.0 * _PWM_FREQ_HZ)) - 1)
        self._bus.write_byte_data(_PCA9685_ADDR, _MODE1, 0x10)          # SLEEP
        self._bus.write_byte_data(_PCA9685_ADDR, _PRESCALE, prescale)   # frekans
        self._bus.write_byte_data(_PCA9685_ADDR, _MODE1, 0x00)          # WAKE
        time.sleep(0.005)
        self._bus.write_byte_data(_PCA9685_ADDR, _MODE1, 0xA0)          # AI + RESTART

    def set_us(self, channel: int, pulse_us: float) -> None:
        """Bir kanala darbe genişliği uygular. Bus yoksa yalnız loglar (no-op)."""
        counts = _us_to_counts(pulse_us)
        if self._bus is None:
            self._log(f"[PCA9685 yok] CH{channel} <- {pulse_us:.0f} us ({counts} sayim)")
            return
        base = _LED0_ON_L + 4 * channel
        self._bus.write_byte_data(_PCA9685_ADDR, base + 0, 0)
        self._bus.write_byte_data(_PCA9685_ADDR, base + 1, 0)
        self._bus.write_byte_data(_PCA9685_ADDR, base + 2, counts & 0xFF)
        self._bus.write_byte_data(_PCA9685_ADDR, base + 3, counts >> 8)

    def close(self) -> None:
        if self._bus is not None:
            try:
                self._bus.close()
            except Exception:
                pass
            self._bus = None


class RealSeparationMechanism(MockSeparationMechanism):
    """
    Gerçek ayrılma mekanizması: iki zıt servoyu (CH14/CH13) PCA9685'ten EŞZAMANLI
    açar. Mantıksal durumu (left/right pozisyon, released, locked) mock'tan miras
    alır; yalnız hareket komutlarında ek olarak fiziksel PWM yazar. Mikroswitch yok
    → `released` komut-tabanlı (mock ile aynı). Kurulumda servolar LOCKED'a alınır.
    """

    def __init__(self, pca: Pca9685) -> None:
        super().__init__()
        self._pca = pca
        # Güvenli (LOCKED) darbeler open()->to_safe() içinde yazılır (bus açıldıktan
        # SONRA). Constructor'da yazmak, bus henüz yokken kafa karıştırıcı no-op
        # ('[PCA9685 yok]') logları üretir; gereksiz.

    def release(self) -> Result[None]:
        # Önce fiziksel PWM: iki servoyu art arda (minimum skew) OPEN'a al.
        self._pca.set_us(CH_SEP_LEFT, _SEP_LEFT_US["open"])
        self._pca.set_us(CH_SEP_RIGHT, _SEP_RIGHT_US["open"])
        # Sonra mantıksal durum + geri bildirim (mock semantiği).
        return super().release()

    def to_safe(self) -> Result[None]:
        # Ayrılma FİZİKSEL ve geri dönüşsüz: bir kez released olduysa servoları
        # LOCKED'a GERİ SÜRME. Aksi halde (örn. kapanışta enter_safe_state) ayrılmış
        # mekanizmayı kapatmaya çalışır ve servolar KENDİ KENDİNE kilitli konuma döner.
        # Yalnız henüz ayrılmadıysa güvenli-kilit yaz (kol mekanizmasıyla aynı semantik).
        if not self.released:
            self._pca.set_us(CH_SEP_LEFT, _SEP_LEFT_US["locked"])
            self._pca.set_us(CH_SEP_RIGHT, _SEP_RIGHT_US["locked"])
        return super().to_safe()


class RealArmMechanism(MockArmMechanism):
    """
    Gerçek kanat/kol mekanizması: CH15 servosunu PCA9685'ten açar. Limit-switch yok
    → `locked` komut-tabanlı (mock ile aynı). Ayrılma FİZİKSEL ve geri dönüşsüz
    olduğundan bir kez açılınca to_safe() kanadı GERİ KATLAMAZ (locked-in-place).
    """

    def __init__(self, pca: Pca9685) -> None:
        super().__init__()
        self._pca = pca
        # Kapalı (LOCKED) darbe open()->to_safe() içinde yazılır (bus açıldıktan sonra).

    def deploy_and_lock(self) -> Result[None]:
        self._pca.set_us(CH_WINGS, _WINGS_US["open"])
        return super().deploy_and_lock()

    def to_safe(self) -> Result[None]:
        # Açılmadıysa kapalı tut; açıldıysa dokunma (geri katlama yok — güvenlik).
        if not self.deployed:
            self._pca.set_us(CH_WINGS, _WINGS_US["locked"])
        return super().to_safe()


class RealApamServo(MockServo):
    """
    Gerçek APAM paraşüt servosu: CH_APAM'ı PCA9685'ten sürer. Mantıksal pozisyonu
    (CLOSED/OPEN) mock'tan miras alır; yalnız hareket komutlarında fiziksel PWM
    yazar. Güvenli konum CLOSED (paraşüt kapalı) — yanlışlıkla açılmaz. Paraşüt
    açma Şartname G-10: failsafe.execute_apam() motorları kill EDİP sonra move_to(OPEN)
    çağırır; burada OPEN → pimi çeken µs uygulanır.
    """

    def __init__(self, pca: Pca9685) -> None:
        super().__init__("apam", ServoPosition.CLOSED)
        self._pca = pca
        # Güvenli (CLOSED) darbe open()->to_safe() içinde yazılır (bus açıldıktan sonra).

    def move_to(self, position: ServoPosition) -> Result[None]:
        us = _APAM_US["open"] if position is ServoPosition.OPEN else _APAM_US["closed"]
        self._pca.set_us(CH_APAM, us)
        return super().move_to(position)

    def to_safe(self) -> Result[None]:
        self._pca.set_us(CH_APAM, _APAM_US["closed"])
        return super().to_safe()


class RealActuatorSuite(ActuatorSuite):
    """
    FLIGHT/HIL aktüatör suite'i: ayrılma + kanat + APAM paraşüt servosu GERÇEK
    PCA9685'ten sürülür; motor ve buzzer mock kalır (motor Pixhawk/DO_MOTOR_TEST
    üzerinden ayrı). Tek PCA9685 bus'ı tüm servoları paylaşır. `open()` donanımı açar.
    """

    def __init__(self, log=print) -> None:
        super().__init__()                       # mock motors/buzzer
        self._pca = Pca9685(log=log)
        self.separation = RealSeparationMechanism(self._pca)
        self.arms = RealArmMechanism(self._pca)
        self.apam_servo = RealApamServo(self._pca)   # paraşüt servosu (mock yerine gerçek)

    def open(self, safe: bool = True) -> Result[None]:
        """
        PCA9685'i açar. `safe=True` ise servoları güvenli konuma alır (ayrılma/kanat
        LOCKED, paraşüt CLOSED). `safe=False` (--keep-servos) ise servolara DOKUNMAZ
        → tezgahta bench tool ile önceden açılmış servolar boot'ta kilitlenmez.
        Donanım yoksa SESSİZCE geçmez: açık hata döndürür ama suite yaşamaya devam eder.
        """
        if self._pca.open():
            if safe:
                # Bus açıldıktan sonra güvenli darbeleri yaz (kurulumda no-op'tular).
                self.separation.to_safe()
                self.arms.to_safe()
                self.apam_servo.to_safe()
            return Result.ok(None)
        from src.common.result import ErrorCode
        return Result.err(ErrorCode.UNAVAILABLE,
                          self._pca.error or "PCA9685 açılamadı")

    def close(self) -> None:
        self._pca.close()
