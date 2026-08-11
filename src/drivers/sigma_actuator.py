"""
Görevi        : SİGMA itki motoru YER-TESTİ aktüatörü (QR tezgah demosu). Yer
                istasyonundan 'SIGMA' komutu gelince Mini Pix/ArduPilot'a TEK bir
                MAV_CMD_DO_MOTOR_TEST gönderir; ArduPilot 4 motoru SIRAYLA, düşük
                gazda, kısa süre döndürür ve otomatik durdurur. Motorları uçuş
                moduna ARM ETMEZ, GPS/EKF gerektirmez → tezgahta (masada) güvenli.
Neden Gerekli : QR Etabı "SİGMA mekanizması tetiklenecek" görevi. QR'da konum/GPS
                yok; GUIDED/arm mümkün değil. DO_MOTOR_TEST, motoru döndürmenin
                GPS'siz ve arm'sız tek güvenli yoludur (tools/motor_bench_test.py ile
                aynı primitif). Bu aktüatör onu FSW komut akışına bağlar: telemetri/
                ARAS/attitude-sim çalışırken yer istasyonundan tetiklenir.
İlişkiler     : apam_actuator.py ile AYNI idiom (gömülü MAVLink sabitleri, connect_fn
                enjeksiyonu, command_long_send + COMMAND_ACK best-effort). FLIGHT'ta
                MavlinkSource'un AÇIK bağlantısını paylaşır (tek seri port iki kez
                açılamaz). config.control.sigma_test_* okur; config.is_simulation'da
                yalnız log basar (donanıma dokunmaz).
DÜRÜSTLÜK NOTU: Gerçek MAVLink G/Ç fiziksel Mini Pix ve pymavlink gerektirir; bu
                ortamda cihaz YOKTUR. SIMULATION_ONLY'de gerçek komut GÖNDERİLMEZ,
                yalnız log. pymavlink/port yoksa sessizce ÇÖKMEZ; açık UNAVAILABLE/
                IO_ERROR döner. ACK best-effort (non-blocking) okunur ki ana döngü
                (1 Hz telemetri) bloklanmasın; ACK yoksa komut yine de gönderilmiştir.
Nasıl Test    : tests/test_sigma_actuator.py — sahte mavlink bağıyla: tetikte tek
                DO_MOTOR_TEST, param eşlemesi (percent tipi, motor sayısı, sıra),
                sim modunda yalnız log, bağlantı hatasında açık hata, tekrar tetik.
GÜVENLİK      : PERVANESİZ çalıştır; motor kollarını sabitle; gaz düşük (config).
"""
from __future__ import annotations

from typing import Callable, Optional

from config.default import AppConfig
from src.common.result import ErrorCode, Result

# MAVLink sabitleri (pymavlink olmadan da test/log için gömülü — apam_actuator deseni).
MAV_CMD_DO_MOTOR_TEST = 209          # ArduPilot: motorları belirli gazda test et
MAV_CMD_COMPONENT_ARM_DISARM = 400   # param1=0 → DISARM (motorları kes)
MOTOR_TEST_THROTTLE_PERCENT = 0      # param2: gaz tipi = yüzde (0..100)
MOTOR_TEST_ORDER_SEQUENCE = 1        # param6: motorları sıra numarasına göre test et
MOTOR_COUNT = 4                      # SİGMA 4× fırçasız motor (PDR: EMAX ECO II 2207)
SAFE_MAX_PERCENT = 20.0              # tezgah güvenliği: gaz üst sınırı
MAV_RESULT_ACCEPTED = 0              # COMMAND_ACK.result kabul kodu

ConnectFn = Callable[[str, int], object]


def _default_connect(port: str, baud: int) -> object:
    """Varsayılan bağlantı: pymavlink mavutil ile seri MAVLink bağı (lazy import)."""
    from pymavlink import mavutil  # type: ignore  # yalnız donanım profilinde kurulu

    return mavutil.mavlink_connection(port, baud=baud)


class SigmaMotorActuator:
    """
    'SIGMA' komutuyla tetiklenen motor yer-testi aktüatörü. `trigger()` her
    çağrıldığında (yeni komut geldikçe) motorları düşük gazda sırayla döndürür.
    FLIGHT'ta paylaşılan MAVLink bağını connect_fn ile alır; SIMULATION'da yalnız log.
    """

    def __init__(self, config: AppConfig,
                 log: Optional[Callable[[str], None]] = None,
                 connect_fn: Optional[ConnectFn] = None) -> None:
        self._cfg = config
        self._pix = config.pixhawk
        self._ctrl = config.control
        self._simulate = config.is_simulation
        self._log = log if log is not None else (lambda s: None)
        self._connect_fn = connect_fn if connect_fn is not None else _default_connect
        self._conn: object | None = None
        self._fire_count = 0

    @property
    def fire_count(self) -> int:
        """Kaç kez tetiklendi (komut gönderildi)."""
        return self._fire_count

    def _percent(self) -> float:
        return max(0.0, min(SAFE_MAX_PERCENT, self._ctrl.sigma_test_percent))

    def trigger(self) -> Result[None]:
        """
        SİGMA motor testini bir kez tetikler: 4 motoru düşük gazda sırayla döndür.
        Sim ise yalnız log; FLIGHT ise gerçek DO_MOTOR_TEST gönderir.
        """
        pct, secs = self._percent(), self._ctrl.sigma_test_seconds
        self._fire_count += 1

        if self._simulate:
            self._log(f"SIGMA ACTUATOR [SIM]: DO_MOTOR_TEST %{pct:.0f} x{MOTOR_COUNT} "
                      f"motor, {secs:.1f} sn/motor — simülasyon, gerçek komut gönderilmedi")
            return Result.ok(None)

        conn = self._ensure_connection()
        if conn.is_err:
            self._log(f"SIGMA ACTUATOR: bağlantı yok — komut gönderilemedi ({conn.message})")
            return conn
        return self._send(conn.unwrap(), pct, secs)

    def stop(self) -> Result[None]:
        """
        Gerçek motorları DURDUR (Şartname G-10: APAM/paraşütten hemen önce).
        Her motora DO_MOTOR_TEST %0 gönderir (aktif motor-testini keser) ve DISARM
        eder. Sim ise yalnız log. ACK best-effort (ana döngüyü bloklamaz).
        """
        if self._simulate:
            self._log("SIGMA ACTUATOR [SIM]: motor STOP (DO_MOTOR_TEST %0 + DISARM) "
                      "— simülasyon, gerçek komut gönderilmedi")
            return Result.ok(None)

        conn = self._ensure_connection()
        if conn.is_err:
            self._log(f"SIGMA ACTUATOR: motor STOP — bağlantı yok ({conn.message})")
            return conn
        c = conn.unwrap()
        try:
            for m in range(1, MOTOR_COUNT + 1):
                c.mav.command_long_send(
                    c.target_system, c.target_component, MAV_CMD_DO_MOTOR_TEST, 0,
                    float(m), float(MOTOR_TEST_THROTTLE_PERCENT), 0.0, 0.0, 1.0, 0.0, 0.0)
            c.mav.command_long_send(
                c.target_system, c.target_component, MAV_CMD_COMPONENT_ARM_DISARM, 0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        except Exception as exc:  # pragma: no cover - donanıma özgü G/Ç hataları
            self._log(f"SIGMA ACTUATOR: motor STOP gönderilemedi: {exc}")
            return Result.err(ErrorCode.IO_ERROR, f"motor STOP gönderilemedi: {exc}")
        self._log("SIGMA ACTUATOR: motor STOP gönderildi (%0 x4 + DISARM)")
        return Result.ok(None)

    def _ensure_connection(self) -> Result[object]:
        if self._conn is not None:
            return Result.ok(self._conn)
        try:
            self._conn = self._connect_fn(self._pix.port, self._pix.baud)
        except ImportError:
            return Result.err(ErrorCode.UNAVAILABLE,
                              "pymavlink kurulu değil (donanım profili gerektirir)")
        except Exception as exc:  # pragma: no cover - donanıma özgü G/Ç hataları
            return Result.err(ErrorCode.IO_ERROR,
                              f"MAVLink bağlantısı açılamadı ({self._pix.port}): {exc}")
        return Result.ok(self._conn)

    def _send(self, conn: object, pct: float, secs: float) -> Result[None]:
        """
        TEK DO_MOTOR_TEST gönderir: motor 1'den başlayıp MOTOR_COUNT motoru SIRAYLA
        test et. ArduPilot sekansı kendi üstünde yürütür → RPi ana döngüsü (1 Hz
        telemetri) bloklanmaz. ACK best-effort (non-blocking) okunur.
        """
        try:
            # command_long_send(target_system, target_component, command, confirmation,
            #   p1=başlangıç motoru, p2=gaz tipi(%), p3=gaz, p4=süre/motor,
            #   p5=motor sayısı, p6=test sırası, p7=0)
            conn.mav.command_long_send(
                conn.target_system, conn.target_component,
                MAV_CMD_DO_MOTOR_TEST, 0,
                1.0, float(MOTOR_TEST_THROTTLE_PERCENT), float(pct), float(secs),
                float(MOTOR_COUNT), float(MOTOR_TEST_ORDER_SEQUENCE), 0.0)
        except Exception as exc:  # pragma: no cover - donanıma özgü G/Ç hataları
            self._log(f"SIGMA ACTUATOR: komut gönderilemedi: {exc}")
            return Result.err(ErrorCode.IO_ERROR, f"DO_MOTOR_TEST gönderilemedi: {exc}")

        # Non-blocking ACK: gelmişse raporla, yoksa döngüyü bekletme (komut gitti).
        ack = conn.recv_match(type="COMMAND_ACK", blocking=False)
        result = getattr(ack, "result", None) if ack is not None else None
        if result == MAV_RESULT_ACCEPTED:
            self._log(f"SIGMA ACTUATOR: DO_MOTOR_TEST ACCEPTED — %{pct:.0f}, "
                      f"{MOTOR_COUNT} motor sırayla {secs:.1f} sn (PERVANESİZ doğrula)")
        else:
            self._log(f"SIGMA ACTUATOR: DO_MOTOR_TEST gönderildi (%{pct:.0f}, "
                      f"{MOTOR_COUNT} motor); ACK henüz yok — motor dönüşünü GÖZLE")
        return Result.ok(None)
