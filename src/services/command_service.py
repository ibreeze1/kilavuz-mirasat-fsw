"""
Görevi        : Yer istasyonundan gelen uplink komutlarını ayrıştırır, doğrular ve
                yönlendirir. Manuel ayrılma ve manuel APAM komutlarını latch'ler;
                6 haneli RHRHRH şifresini S2D-IOT servisine iletir.
Neden Gerekli : Gereksinim-7 (manuel ayrılma), Gereksinim-10 (manuel APAM),
                §2.3 (BONUS-2 komut alma). Komutlar güvenlik-kritik olduğundan
                doğrulanmadan uygulanmamalı; bilinmeyen komut sessizce yutulmamalı.
İlişkiler     : Ana döngü, çevrim başına bekleyen komutları handle() ile işler ve
                latch'lenen bayrakları FlightContext'e (manual_separation_cmd,
                manual_apam_cmd) aktarır. RHRHRH için S2dIotService kullanılır.
Nasıl Test    : tests/test_command_service.py — manuel sep/apam latch, RHRHRH
                yönlendirme, bilinmeyen/geçersiz komut, latch kalıcılığı.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from src.common.result import ErrorCode, Result
from src.services.s2d_iot import S2dIotService

_SEPARATION_ALIASES = {"SEP", "MANUAL_SEP", "MANUAL_SEPARATION", "AYIR"}
_APAM_ALIASES = {"APAM", "MANUAL_APAM", "PARASUT", "PARACHUTE"}
_SIGMA_ALIASES = {"SIGMA", "SIGMA_TEST", "MOTOR", "MOTOR_TEST", "SIGMA_TETIK"}


class CommandKind(Enum):
    MANUAL_SEPARATION = "MANUAL_SEPARATION"
    MANUAL_APAM = "MANUAL_APAM"
    MANUAL_SIGMA = "MANUAL_SIGMA"
    S2D_IOT = "S2D_IOT"


@dataclass(frozen=True)
class CommandResult:
    kind: CommandKind
    detail: str


class CommandService:
    """
    Uplink komut yorumlayıcı. Manuel ayrılma/APAM bayrakları LATCH'lenir: bir kez
    komut alınınca isteğe bağlı geri alınamaz (güvenli taraf — operatör kararı
    kalıcıdır). Bilinmeyen komutlar açık hata döndürür.
    """

    def __init__(self, s2d: S2dIotService) -> None:
        self._s2d = s2d
        self._manual_separation = False
        self._manual_apam = False
        self._sigma_count = 0
        self.handled_count = 0

    @property
    def manual_separation_requested(self) -> bool:
        return self._manual_separation

    @property
    def manual_apam_requested(self) -> bool:
        return self._manual_apam

    @property
    def sigma_request_count(self) -> int:
        """
        'SIGMA' komutu kaç kez alındı. Ayrılma/APAM'ın aksine LATCH DEĞİL sayaçtır:
        QR tezgah demosunda motor testi tekrar tekrar tetiklenebilmeli. Ana döngü
        sayaç arttıkça SigmaMotorActuator.trigger() çağırır (kenar tespiti).
        """
        return self._sigma_count

    def handle(self, command: str) -> Result[CommandResult]:
        if not isinstance(command, str) or not command.strip():
            return Result.err(ErrorCode.INVALID_DATA, "boş komut")
        token = command.strip().upper()

        if token in _SEPARATION_ALIASES:
            self._manual_separation = True
            self.handled_count += 1
            return Result.ok(CommandResult(CommandKind.MANUAL_SEPARATION,
                                           "manuel ayrılma latch'lendi"))
        if token in _APAM_ALIASES:
            self._manual_apam = True
            self.handled_count += 1
            return Result.ok(CommandResult(CommandKind.MANUAL_APAM,
                                           "manuel APAM latch'lendi"))
        if token in _SIGMA_ALIASES:
            # QR tezgah demosu: motor yer-testini tetikle (latch değil, sayaç++).
            self._sigma_count += 1
            self.handled_count += 1
            return Result.ok(CommandResult(CommandKind.MANUAL_SIGMA,
                                           f"SİGMA motor testi tetiklendi (#{self._sigma_count})"))
        if len(token) == 6:
            # RHRHRH şifresi olabilir → S2D servisine yönlendir.
            r = self._s2d.process(token)
            if r.is_err:
                return Result.err(r.code, f"S2D komutu reddedildi: {r.message}")
            self.handled_count += 1
            return Result.ok(CommandResult(CommandKind.S2D_IOT,
                                           f"RHRHRH yönlendirildi: {token}"))

        return Result.err(ErrorCode.INVALID_DATA, f"bilinmeyen komut: {command!r}")
