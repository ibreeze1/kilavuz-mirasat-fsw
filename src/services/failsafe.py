"""
Görevi        : Failsafe yöneticisi ve APAM (Acil Paraşüt Açma) mantığı. 16 m/s
                üzeri 10 sn KESİNTİSİZ iniş eşiğini sayaçla izler; iniş fazı ve
                >100 m irtifa kilitlerini uygular; tetiklendiğinde motor kill →
                paraşüt açma sıralamasını üretir. Manuel APAM komutunu da işler.
Neden Gerekli : Şartname Gereksinim-10 s.11 (DOĞRULANMIŞ) + ANA_PROMPT APAM konsepti.
                Uçuş güvenliğinin en kritik parçası; false-trigger önlemleri şart.
İlişkiler     : FlightContext + HealthFlags okur; ActuatorSuite üzerinden motor
                kill ve APAM servosunu sürer; state machine EMERGENCY_APAM'a geçer.
                Link loss TEK BAŞINA APAM tetiklemez.
Nasıl Test    : tests/test_failsafe_apam.py — eşik/sayaç/sıfırlama, irtifa kilidi,
                motor-kill→paraşüt sıralaması, iniş fazı kilidi, link-loss.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from config.default import ApamConfig
from src.common.result import Result
from src.drivers.mock_actuators import ActuatorSuite
from src.hal.interfaces import ServoPosition
from src.mission.context import FlightContext


class ApamTrigger(Enum):
    NONE = "NONE"
    ALGORITHM = "ALGORITHM"     # 16 m/s × 10 sn eşiği
    MANUAL = "MANUAL"           # yer istasyonundan manuel komut


@dataclass
class FailsafeDecision:
    """Bir çevrimin failsafe çıktısı."""

    apam_should_deploy: bool
    apam_active: bool
    trigger: ApamTrigger
    overspeed_timer_s: float
    reason: str = ""


class FailsafeManager:
    """
    APAM durum makinesi. Bir kez tetiklenince (latch) motor kill ve paraşüt
    açma sıralamasını uygular ve aktif kalır. Sıralama KESİN:
        motor kill  →  APAM servosu OPEN
    """

    def __init__(self, config: ApamConfig) -> None:
        self._cfg = config
        self._overspeed_timer_s = 0.0
        self._apam_latched = False
        self._trigger = ApamTrigger.NONE
        self._last_time_s: float | None = None
        self._deployed = False

    @property
    def apam_active(self) -> bool:
        return self._apam_latched

    @property
    def overspeed_timer_s(self) -> float:
        return self._overspeed_timer_s

    def update(self, ctx: FlightContext) -> FailsafeDecision:
        """
        Bir çevrim değerlendirir. `ctx.mission_time_s` monoton artan görev
        zamanıdır; sayaç bu zamanın deltasıyla ilerler (deterministik).
        """
        # dt hesabı (ilk çağrıda 0).
        if self._last_time_s is None:
            dt = 0.0
        else:
            dt = max(0.0, ctx.mission_time_s - self._last_time_s)
        self._last_time_s = ctx.mission_time_s

        c = self._cfg
        # --- İniş fazı kilidi (ASSUMPTION-004): yükselmede APAM tetiklenmez ---
        # Ayrılma gerçekleşmiş ve iniyor olmalı.
        in_descent = ctx.separation_confirmed and (not ctx.ascending)

        # --- Overspeed sayacı: 16 m/s ÜZERİ kesintisiz süre ---
        if in_descent and ctx.descent_speed_mps > c.trigger_speed_mps:
            self._overspeed_timer_s += dt
        elif ctx.descent_speed_mps <= c.safe_speed_reset_mps:
            # Hız güvenli seviyeye düşerse sayaç SIFIRLANIR (anlık artış tetiklemez).
            self._overspeed_timer_s = 0.0

        # --- Çoklu-sensör çelişki filtresi (false-trigger önlemi) ---
        # İkinci bir sensör (GPS) MEVCUT ve aşırı hızı YALANLIYORSA tetikleme.
        # GPS yoksa (gps_valid=False) tek sensöre güvenilir → filtre uygulanmaz
        # (APAM güvenlik gereği yine de tetiklenebilmelidir).
        speed_contradicted = ctx.gps_valid and (not ctx.speed_consistent)

        # --- Tetik koşulları ---
        algo_trigger = (
            in_descent
            and self._overspeed_timer_s >= c.trigger_duration_s
            and ctx.altitude_m > c.min_deploy_altitude_m   # >100 m kilidi
            and not speed_contradicted                     # çoklu-sensör onayı
        )
        # Manuel APAM: operatörün AÇIK kararı → irtifadan BAĞIMSIZ açılır (Şartname
        # G-10 "Acil bir Durumda manuel paraşüt açma komutu gönderilerek
        # aktifleştirilebilmelidir"; irtifa sınırı yalnız ALGORİTMİK tetiktedir).
        # Bu, QR tezgah demosunda (irtifa ~0) 'APAM' komutunun paraşütü açmasını sağlar.
        manual_trigger = ctx.manual_apam_cmd

        should_deploy = False
        reason = ""
        if not self._apam_latched:
            if algo_trigger:
                self._apam_latched = True
                self._trigger = ApamTrigger.ALGORITHM
                should_deploy = True
                reason = (f"16 m/s üzeri {self._overspeed_timer_s:.1f} sn; "
                          f"irtifa {ctx.altitude_m:.0f} m > 100 m")
            elif manual_trigger:
                self._apam_latched = True
                self._trigger = ApamTrigger.MANUAL
                should_deploy = True
                reason = "manuel APAM komutu"

        return FailsafeDecision(
            apam_should_deploy=should_deploy,
            apam_active=self._apam_latched,
            trigger=self._trigger,
            overspeed_timer_s=self._overspeed_timer_s,
            reason=reason,
        )

    def execute_apam(self, actuators: ActuatorSuite) -> Result[None]:
        """
        APAM açma sıralamasını uygular (KESİN sıra):
          1) MOTOR KILL (tüm motorlar 0 throttle / disarm)
          2) APAM servosu ile paraşüt kapağı OPEN
        Paraşüt açmadan hemen önce motorların durması Şartname G-10 gereğidir.
        """
        kill = actuators.motors.kill()
        if kill.is_err:
            return kill
        # Motorların gerçekten durduğunu doğrula (arm interlock zaten koruyor).
        if actuators.motors.throttle != 0.0:
            from src.common.result import ErrorCode
            return Result.err(ErrorCode.SAFETY_INTERLOCK,
                              "motorlar durmadan paraşüt açılamaz")
        deploy = actuators.apam_servo.move_to(ServoPosition.OPEN)
        if deploy.is_err:
            return deploy
        self._deployed = True
        return Result.ok(None)

    @property
    def parachute_deployed(self) -> bool:
        return self._deployed
