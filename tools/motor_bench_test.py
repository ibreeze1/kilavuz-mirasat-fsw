"""
Görevi        : SİGMA itki motorları tezgah bring-up testi (RPi → Mini Pix/ArduPilot
                → ESC). 4 fırçasız motoru (EMAX ECO II 2207) TEK TEK ve düşük gazda
                döndürerek kablolama, dönüş yönü, motor sırası ve ESC'yi doğrular.
                MAV_CMD_DO_MOTOR_TEST kullanır: motorları UÇUŞ MODUNA ARM ETMEDEN,
                GPS/EKF gerektirmeden, ArduPilot'ın kendi güvenli motor-test
                primitifiyle döndürür ve süre sonunda otomatik durdurur.
                Uçuş yazılımından BAĞIMSIZ; yalnız motor/ESC zincirini doğrular.
Neden Gerekli : docs/HARDWARE_BRINGUP — SİGMA itki sistemi (Gereksinim-9 aktif iniş,
                BONUS-1 hover) gerçek motoru döndürmeden doğrulanamaz. FSW throttle'ı
                Mini Pix üzerinden gönderir (send_setpoint, Aşama 5); ama İLK motor
                dönüşü her zaman pervanesiz tezgahta, düşük gazda, kill yoluyla
                yapılmalıdır. Bu betik o güvenli ilk adımdır.
İlişkiler     : config.pixhawk (port/baud/ACK timeout) okumak yerine CLI ile alır;
                apam_actuator.py ile AYNI MAVLink idiomu (gömülü sabitler, connect_fn
                enjeksiyonu, command_long_send + COMMAND_ACK). Gerçek FSW'ye dokunmaz.
DÜRÜSTLÜK NOTU: Gerçek MAVLink G/Ç fiziksel Mini Pix ve `pymavlink` gerektirir; bu
                ortamda cihaz YOKTUR. pymavlink/port yoksa SESSİZCE ÇÖKMEZ; açık hata
                verir ya da --dry-run ile yalnız komut planını yazar (RPi'de gerçek
                doğrulama şart). ArduPilot MAV_CMD_DO_MOTOR_TEST sonucu COMMAND_ACK
                ile teyit edilir; ACK yoksa/RED ise açıkça raporlanır (sessiz başarı YOK).

GÜVENLİK (OKU — motorlar 3-fazlı, yaralar):
  - PERVANELERİ ÇIKAR. İlk tüm testleri MUTLAKA pervanesiz yap.
  - Görev yükünü/motor kollarını masaya SABİTLE; kablo/el motordan uzak dursun.
  - Mini Pix güvenlik switch'i (varsa) OFF/basılı olmalı; yoksa BRD_SAFETY_DEFLT.
  - Düşük yüzde ile başla (varsayılan %5), tek motor, kısa süre. Kademeli artır.
  - Elinin altında ana batarya ayırıcısı (fiziksel kill) BULUNSUN. --stop yazılımsal.
  - ESC'lerin kalibre ve motor sırası/dönüş yönünün ArduPilot'ta doğru olduğunu doğrula.

KULLANIM (RPi'de):
  python tools/motor_bench_test.py --motor 1 --percent 5 --seconds 3   # tek motor, düşük gaz
  python tools/motor_bench_test.py --all --percent 5 --seconds 2       # 4 motoru sırayla
  python tools/motor_bench_test.py --stop                              # acil durdur (disarm + %0)
  python tools/motor_bench_test.py --all --percent 5 --dry-run         # donanımsız komut planı (laptop)
  python tools/motor_bench_test.py --motor 2 --percent 8 --port /dev/ttyACM0 --baud 115200
"""
from __future__ import annotations

import argparse
import sys
import time
from typing import Callable, Optional

# ─────────────────────────── AYARLANACAK VARSAYILANLAR ──────────────────────
DEFAULT_PORT = "/dev/ttyACM0"     # RPi ↔ Mini Pix (config.pixhawk ile aynı varsayılan)
DEFAULT_BAUD = 115200
MOTOR_COUNT = 4                   # SİGMA 4× fırçasız motor (PDR: EMAX ECO II 2207)
ACK_TIMEOUT_S = 2.0               # COMMAND_ACK bekleme üst sınırı
HEARTBEAT_TIMEOUT_S = 10.0        # ilk heartbeat bekleme (bağlantı doğrulama)
SAFE_MAX_PERCENT = 20.0          # betiğin izin verdiği üst gaz sınırı (tezgah güvenliği)
GAP_S = 1.0                       # --all modunda motorlar arası bekleme
# ─────────────────────────────────────────────────────────────────────────────

# MAVLink sabitleri (pymavlink olmadan da test/log için gömülü; enum değerleri
# MAVLink standardıdır — apam_actuator.py ile aynı yaklaşım).
MAV_CMD_DO_MOTOR_TEST = 209        # ArduPilot: tek motoru belirli gazda test et
MAV_CMD_COMPONENT_ARM_DISARM = 400  # param1=0 → DISARM
MOTOR_TEST_THROTTLE_PERCENT = 0    # param2: gaz tipi = yüzde (0..100)
MAV_RESULT_ACCEPTED = 0            # COMMAND_ACK.result kabul kodu

# Bağlantı üreticisi: (port, baud) → mavlink bağlantı nesnesi. Test/dry-run bunu
# enjekte ederek gerçek donanım olmadan sahte bir bağlantı verebilir.
ConnectFn = Callable[[str, int], object]


def _default_connect(port: str, baud: int) -> object:
    """Varsayılan bağlantı: pymavlink mavutil ile seri MAVLink bağı (lazy import)."""
    from pymavlink import mavutil  # type: ignore  # yalnız donanım profilinde kurulu

    return mavutil.mavlink_connection(port, baud=baud)


class MotorTester:
    """
    Mini Pix ile MAVLink bağını açar ve MAV_CMD_DO_MOTOR_TEST ile motorları
    tek tek/sırayla düşük gazda döndürür. `open()` çağrılana dek bağlı değildir;
    pymavlink/port yoksa SESSİZCE ÇÖKMEZ (apam_actuator deseni). Dry-run modunda
    donanıma hiç dokunmadan komutları planlar.
    """

    def __init__(self, port: str = DEFAULT_PORT, baud: int = DEFAULT_BAUD,
                 dry_run: bool = False, connect_fn: Optional[ConnectFn] = None,
                 log: Callable[[str], None] = print) -> None:
        self._port = port
        self._baud = baud
        self._dry = dry_run
        self._connect_fn = connect_fn if connect_fn is not None else _default_connect
        self._log = log
        self._conn: object | None = None
        self.error: str | None = None

    # ------------------------------------------------------------- yaşam döngüsü
    def open(self) -> bool:
        if self._dry:
            self._log(f"[DRY] MAVLink {self._port}@{self._baud} — donanım yok, "
                      "yalnız komut planı yazılacak")
            return True
        try:
            self._conn = self._connect_fn(self._port, self._baud)
        except ImportError:
            self.error = "pymavlink kurulu değil (pip install -r requirements-hardware.txt)"
            return False
        except Exception as exc:  # port/IO hataları — açık raporla, çökme yok
            self.error = f"MAVLink bağlantısı açılamadı ({self._port}): {exc}"
            return False
        try:
            hb = self._conn.wait_heartbeat(timeout=HEARTBEAT_TIMEOUT_S)
        except Exception as exc:
            self.error = f"heartbeat beklenirken hata: {exc}"
            return False
        if hb is None:
            self.error = (f"MAVLink heartbeat alınamadı ({HEARTBEAT_TIMEOUT_S:.0f} sn) "
                          "— Mini Pix yanıt vermiyor (kablo/port/baud?)")
            return False
        self._log(f"Bağlandı: Mini Pix heartbeat alındı "
                  f"(sys={self._conn.target_system}, comp={self._conn.target_component})")
        return True

    def _send_and_ack(self, command: int, params: tuple, what: str) -> bool:
        """command_long gönderir, COMMAND_ACK'i bekleyip sonucu loglar (apam deseni)."""
        p = list(params) + [0.0] * (7 - len(params))
        if self._dry or self._conn is None:
            self._log(f"[{'DRY' if self._dry else '??'}] {what}: CMD {command} "
                      f"params={[round(x, 2) for x in p]}")
            return True
        try:
            self._conn.mav.command_long_send(
                self._conn.target_system, self._conn.target_component,
                command, 0, *p)
        except Exception as exc:
            self._log(f"{what}: komut gönderilemedi: {exc}")
            return False
        ack = self._conn.recv_match(type="COMMAND_ACK", blocking=True,
                                    timeout=ACK_TIMEOUT_S)
        if ack is None:
            self._log(f"{what}: COMMAND_ACK zaman aşımı ({ACK_TIMEOUT_S:.1f} sn) "
                      "— komut TEYİT EDİLMEDİ")
            return False
        result = getattr(ack, "result", None)
        if result == MAV_RESULT_ACCEPTED:
            self._log(f"{what}: ACCEPTED (Mini Pix kabul etti)")
            return True
        self._log(f"{what}: REDDEDİLDİ (MAV_RESULT={result}) — güvenlik switch'i? "
                  "ESC kalibre mi? motor sırası?")
        return False

    def test_motor(self, motor: int, percent: float, seconds: float) -> bool:
        """
        Tek motoru `percent` gazda `seconds` süre döndürür (ArduPilot süre sonunda
        otomatik durdurur). motor 1..MOTOR_COUNT (ArduPilot test sırası).
        """
        percent = max(0.0, min(SAFE_MAX_PERCENT, percent))
        self._log(f"MOTOR {motor}: %{percent:.0f} gaz, {seconds:.1f} sn (pervanesiz doğrula!)")
        # param1=motor no, param2=gaz tipi(%%), param3=gaz, param4=süre, param5=1 motor
        return self._send_and_ack(
            MAV_CMD_DO_MOTOR_TEST,
            (float(motor), float(MOTOR_TEST_THROTTLE_PERCENT), float(percent),
             float(seconds), 1.0, 0.0),
            f"DO_MOTOR_TEST(motor={motor}, %{percent:.0f}, {seconds:.1f}s)")

    def test_all(self, percent: float, seconds: float,
                 count: int = MOTOR_COUNT) -> int:
        """Tüm motorları SIRAYLA (tek tek) test eder. Kaç motorun ACCEPTED aldığını döndürür."""
        ok = 0
        for m in range(1, count + 1):
            if self.test_motor(m, percent, seconds):
                ok += 1
            # ArduPilot süre sonunda durdurur; motorlar arası kısa bekleme + gözlem.
            if not self._dry:
                time.sleep(seconds + GAP_S)
        self._log(f"ÖZET: {ok}/{count} motor komutu ACCEPTED. Her motorda dönüş "
                  "YÖNÜNÜ ve TİTREŞİMİ ELLE doğrula (video öner).")
        return ok

    def stop(self) -> bool:
        """Acil durdur: her motora %0 test + DISARM (yazılımsal). Fiziksel kill ayrı!"""
        self._log("STOP: tüm motorlara %0 + DISARM gönderiliyor...")
        for m in range(1, MOTOR_COUNT + 1):
            self._send_and_ack(
                MAV_CMD_DO_MOTOR_TEST,
                (float(m), float(MOTOR_TEST_THROTTLE_PERCENT), 0.0, 0.0, 1.0, 0.0),
                f"DO_MOTOR_TEST(motor={m}, %0)")
        return self._send_and_ack(
            MAV_CMD_COMPONENT_ARM_DISARM, (0.0,), "ARM_DISARM(DISARM)")

    def close(self) -> None:
        if self._conn is not None:
            try:
                self._conn.close()
            except Exception:
                pass


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(
        description="SİGMA itki motorları tezgah bring-up testi (Mini Pix/ArduPilot)")
    p.add_argument("--motor", type=int, help="Tek motoru test et (1..%d)" % MOTOR_COUNT)
    p.add_argument("--all", action="store_true", help="Tüm motorları sırayla test et")
    p.add_argument("--percent", type=float, default=5.0,
                   help="Gaz yüzdesi (0..%d, varsayılan 5)" % int(SAFE_MAX_PERCENT))
    p.add_argument("--seconds", type=float, default=3.0, help="Her motor için süre (sn)")
    p.add_argument("--stop", action="store_true", help="Acil durdur: %%0 + DISARM")
    p.add_argument("--port", default=DEFAULT_PORT, help="MAVLink portu")
    p.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="MAVLink baud")
    p.add_argument("--dry-run", action="store_true",
                   help="Donanımsız: yalnız komut planını yaz (laptop)")
    args = p.parse_args(argv)

    if not (args.motor or args.all or args.stop):
        p.print_help()
        print("\nHATA: --motor N | --all | --stop ver.", file=sys.stderr)
        return 2
    if args.motor is not None and not (1 <= args.motor <= MOTOR_COUNT):
        print(f"HATA: --motor 1..{MOTOR_COUNT} olmalı.", file=sys.stderr)
        return 2

    tester = MotorTester(port=args.port, baud=args.baud, dry_run=args.dry_run)
    if not tester.open():
        print(f"HATA: {tester.error}", file=sys.stderr)
        print("İpucu: RPi'de pymavlink kurulu mu, Mini Pix bağlı mı "
              "(ls /dev/ttyACM*), baud doğru mu? Laptopta --dry-run kullan.",
              file=sys.stderr)
        return 1

    try:
        if args.stop:
            return 0 if tester.stop() else 1
        if args.all:
            ok = tester.test_all(args.percent, args.seconds)
            return 0 if ok == MOTOR_COUNT else 1
        ok = tester.test_motor(args.motor, args.percent, args.seconds)
        return 0 if ok else 1
    finally:
        # Güvenlik: çıkışta motorları durdurmayı DENE (dry-run'da yalnız loglar).
        if not args.dry_run:
            tester.stop()
        tester.close()


if __name__ == "__main__":
    raise SystemExit(main())
