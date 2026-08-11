"""
Görevi        : APAM (Acil Paraşüt Açma) servosu tezgah testi (RPi + PCA9685).
                Paraşüt kapağı pimini çeken servoyu CLOSED (güvenli) ↔ OPEN (paraşüt
                bırak) konumları arasında sürer. Uçuş yazılımından BAĞIMSIZ; yalnız
                mekanizmayı doğrulamak/kalibre etmek içindir.
Neden Gerekli : Şartname Gereksinim-10 (APAM) + QR "APAM mekanizması tetiklenecek".
                Paraşüt servosu ayrılma/kanat gibi PCA9685'te (I²C PWM); FSW
                RealApamServo ile sürer ama İLK doğrulama/kalibrasyon tezgahta bu
                betikle yapılır (separation_bench_test.py ile aynı yaklaşım).
İlişkiler     : smbus2 ile PCA9685'e (0x40) doğrudan PWM yazar. src/drivers/
                real_actuators.py'deki CH_APAM ve _APAM_US ile AYNI değerler olmalı
                (kalibrasyon burada bulunur, oraya taşınır).
DÜRÜSTLÜK NOTU: Gerçek I²C/PWM fiziksel PCA9685 ve smbus2 gerektirir; bu ortamda
                cihaz YOKTUR. smbus2/bus yoksa SESSİZCE ÇÖKMEZ; açık hata verir ya da
                --dry-run ile yalnız komut planını yazar (RPi'de gerçek doğrulama şart).

GÜVENLİK:
  - Paraşütü hazneye YERLEŞTİRMEDEN önce servo yönünü/uçlarını doğrula (boş test).
  - Önce CLOSED↔OPEN uçlarını DÜŞÜK açı farkıyla dene; mekaniği/pimi zorlama.
  - Betik başlangıçta servoyu GÜVENLİ (CLOSED) konuma alır. OPEN paraşütü BIRAKIR.

KULLANIM (RPi'de):
  python tools/apam_bench_test.py --close                 # güvenli (kapalı) konuma al
  python tools/apam_bench_test.py --open                  # paraşütü BIRAK (aç)
  python tools/apam_bench_test.py --channel 12 --us 1500  # elle konumla (kalibrasyon)
  python tools/apam_bench_test.py --open --dry-run        # donanımsız komut planı (laptop)

AYAR: CH_APAM ve CLOSED/OPEN µs'yi KENDİ mekaniğine göre ölç; real_actuators.py ile eşitle.
"""
from __future__ import annotations

import argparse
import sys
import time

# ─────────────────────────── AYARLANACAK SABİTLER ───────────────────────────
CH_APAM = 12                 # paraşüt servosu PCA9685 kanalı (13/14/15 ayrılma/kanat)
APAM_US = {"closed": 2100, "open": 1000}   # kalibre (2026-08-11): kapalı YÜKSEK, açık DÜŞÜK
SETTLE_S = 0.6               # hareketin oturması için bekleme

# PCA9685 I²C
I2C_BUS = 1
PCA9685_ADDR = 0x40
PWM_FREQ_HZ = 50.0
# ─────────────────────────────────────────────────────────────────────────────

_MODE1 = 0x00
_PRESCALE = 0xFE
_LED0_ON_L = 0x06
_PERIOD_US = 1_000_000.0 / PWM_FREQ_HZ


def _us_to_counts(pulse_us: float) -> int:
    counts = int(round(4096.0 * pulse_us / _PERIOD_US))
    return max(0, min(4095, counts))


class Pca9685:
    """Minimal PCA9685 servo sürücüsü (smbus2). separation_bench_test.py ile aynı."""

    def __init__(self, dry_run: bool = False, log=print) -> None:
        self._dry = dry_run
        self._log = log
        self._bus = None
        self.error: str | None = None

    def open(self) -> bool:
        if self._dry:
            self._log(f"[DRY] PCA9685 @0x{PCA9685_ADDR:02X}, {PWM_FREQ_HZ:.0f} Hz (donanım yok)")
            return True
        try:
            from smbus2 import SMBus  # type: ignore  # yalnız RPi/donanım profilinde
        except ImportError:
            self.error = "smbus2 kurulu değil (pip install -r requirements-hardware.txt)"
            return False
        try:
            self._bus = SMBus(I2C_BUS)
            self._init_chip()
        except Exception as exc:
            self.error = f"PCA9685 açılamadı (I2C-{I2C_BUS} @0x{PCA9685_ADDR:02X}): {exc}"
            return False
        return True

    def _init_chip(self) -> None:
        prescale = int(round(25_000_000.0 / (4096.0 * PWM_FREQ_HZ)) - 1)
        self._bus.write_byte_data(PCA9685_ADDR, _MODE1, 0x10)
        self._bus.write_byte_data(PCA9685_ADDR, _PRESCALE, prescale)
        self._bus.write_byte_data(PCA9685_ADDR, _MODE1, 0x00)
        time.sleep(0.005)
        self._bus.write_byte_data(PCA9685_ADDR, _MODE1, 0xA0)

    def set_us(self, channel: int, pulse_us: float) -> None:
        counts = _us_to_counts(pulse_us)
        if self._dry or self._bus is None:
            self._log(f"[{'DRY' if self._dry else '??'}] CH{channel} <- {pulse_us:.0f} us "
                      f"({counts} sayim)")
            return
        base = _LED0_ON_L + 4 * channel
        self._bus.write_byte_data(PCA9685_ADDR, base + 0, 0)
        self._bus.write_byte_data(PCA9685_ADDR, base + 1, 0)
        self._bus.write_byte_data(PCA9685_ADDR, base + 2, counts & 0xFF)
        self._bus.write_byte_data(PCA9685_ADDR, base + 3, counts >> 8)

    def close(self) -> None:
        if self._bus is not None:
            try:
                self._bus.close()
            except Exception:
                pass


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="APAM paraşüt servosu tezgah testi (RPi/PCA9685)")
    p.add_argument("--close", action="store_true", help="Servoyu güvenli (CLOSED) konuma al")
    p.add_argument("--open", action="store_true", help="Paraşütü BIRAK (OPEN)")
    p.add_argument("--channel", type=int, help="Tek kanala elle PWM (--us ile, kalibrasyon)")
    p.add_argument("--us", type=float, help="Elle darbe genişliği (mikrosaniye)")
    p.add_argument("--dry-run", action="store_true", help="Donanımsız: yalnız komut planını yaz")
    args = p.parse_args(argv)

    if not (args.close or args.open or args.channel is not None):
        p.print_help()
        print("\nHATA: --close | --open | --channel N --us X ver.", file=sys.stderr)
        return 2

    pca = Pca9685(dry_run=args.dry_run)
    if not pca.open():
        print(f"HATA: {pca.error}", file=sys.stderr)
        print("İpucu: RPi'de I2C açık mı ('i2cdetect -y 1' 0x40 görmeli), smbus2 kurulu mu? "
              "Laptopta --dry-run kullan.", file=sys.stderr)
        return 1

    # Elle konumlama modunda servo OLDUĞU GİBİ bırakılır (kalibrasyon için).
    manual = args.channel is not None
    try:
        if manual:
            if args.us is None:
                print("HATA: --channel ile --us de vermelisin.", file=sys.stderr)
                return 2
            pca.set_us(args.channel, args.us)
            print(f"CH{args.channel} <- {args.us:.0f} us uygulandı ve KONUMDA BIRAKILDI.")
            return 0

        if args.open:
            print("APAM: paraşüt BIRAKILIYOR (OPEN)...")
            pca.set_us(CH_APAM, APAM_US["open"])
            time.sleep(SETTLE_S)
            print("APAM OPEN uygulandı — paraşüt kapağı açıldı mı, pim çekildi mi KONTROL ET.")
            return 0

        # --close (varsayılan güvenli)
        pca.set_us(CH_APAM, APAM_US["closed"])
        print("APAM güvenli (CLOSED) konumda.")
        return 0
    finally:
        pca.close()


if __name__ == "__main__":
    raise SystemExit(main())
