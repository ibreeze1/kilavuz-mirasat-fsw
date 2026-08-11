"""
Görevi        : Profil bazlı sürücü fabrikası. Çalışma profiline göre mock veya
                gerçek telemetri linkini seçer ve FLIGHT/HIL profilinde donanım
                desteğinin varlığını denetler.
Neden Gerekli : SIMULATION_ONLY masaüstünde mock sürücülerle çalışır; FLIGHT/HIL
                gerçek donanım sürücüleri gerektirir. Yanlış profilde sessizce mock'a
                düşmek güvenlik riskidir → açık kapı (gate) gerekir.
İlişkiler     : Ana döngü telemetri linkini bu fabrikadan alabilir. Gerçek sürücüler
                (real_lora) donanım kütüphaneleri mevcutsa kullanılır.
Nasıl Test    : tests/test_driver_factory.py — SIMULATION→mock; FLIGHT donanım
                yokken açık hata; gerçek link güvenli degrade.
"""
from __future__ import annotations

import importlib.util

from config.default import AppConfig, RunProfile
from src.common.result import ErrorCode, Result
from src.drivers.mavlink_source import (
    MavlinkBarometer,
    MavlinkBattery,
    MavlinkFlightControllerLink,
    MavlinkGps,
    MavlinkImu,
    MavlinkSource,
)
from src.drivers.mock_sensors import (
    MockBarometer,
    MockBattery,
    MockGps,
    MockImu,
    MockTelemetryLink,
)
from src.drivers.real_lora import RealLoraE22Link
from src.drivers.sim_flight_controller import SimulatedFlightControllerLink

# FLIGHT/HIL için gerekli donanım kütüphaneleri (varlık kontrolü; import etmez).
# pyserial: LoRa/GPS UART; pymavlink: Mini Pix MAVLink kaynağı (mavlink_source).
_HARDWARE_MODULES = ("serial", "pymavlink")


def hardware_libs_available() -> bool:
    """Gerekli donanım kütüphaneleri kurulu mu (import etmeden kontrol)."""
    return all(importlib.util.find_spec(m) is not None for m in _HARDWARE_MODULES)


def check_profile_runnable(config: AppConfig) -> Result[None]:
    """
    Profilin bu ortamda çalıştırılabilir olduğunu doğrular. SIMULATION her zaman
    çalışır; FLIGHT/HIL donanım kütüphaneleri yoksa açık hata döndürür.
    """
    if config.profile is RunProfile.SIMULATION_ONLY:
        return Result.ok(None)
    if not hardware_libs_available():
        return Result.err(
            ErrorCode.UNAVAILABLE,
            f"{config.profile.value} profili donanım kütüphanelerini gerektirir "
            f"(eksik: {[m for m in _HARDWARE_MODULES if importlib.util.find_spec(m) is None]}). "
            "requirements-hardware.txt kurulmalı ve gerçek donanımda çalıştırılmalı.")
    return Result.ok(None)


def create_telemetry_link(config: AppConfig, port: str | None = None):
    """
    Profile göre telemetri linki döndürür. SIMULATION → MockTelemetryLink;
    FLIGHT/HIL → RealLoraE22Link (henüz açık değil; open() donanımda çağrılır).
    Port verilmezse config.telemetry.lora_port kullanılır.
    """
    if config.profile is RunProfile.SIMULATION_ONLY:
        return MockTelemetryLink()
    return RealLoraE22Link(port or config.telemetry.lora_port, config.telemetry)


def create_mavlink_source(config: AppConfig, clock, connect_fn=None):
    """
    Mini Pix MAVLink veri kaynağı döndürür. SIMULATION_ONLY'de None (mock sensörler
    kullanılır); FLIGHT/HIL'de MavlinkSource (henüz açık değil; open() çağrılır).
    Tek kaynak hem sensör adaptörlerini besler hem de bağlantıyı ApamActuator ile
    paylaşır (tek seri port iki kez açılamaz).
    """
    if config.profile is RunProfile.SIMULATION_ONLY:
        return None
    return MavlinkSource(config.pixhawk, clock, connect_fn=connect_fn)


def create_sensors(config: AppConfig, clock, profile, mission_time, source=None):
    """
    (baro, imu, gps, batt) dörtlüsü döndürür. SIMULATION_ONLY → mock sürücüler;
    FLIGHT/HIL → Mini Pix MAVLink adaptörleri (tek `source`'tan okur).
    """
    if config.profile is RunProfile.SIMULATION_ONLY:
        return (MockBarometer(clock, profile, mission_time),
                MockImu(clock, profile, mission_time),
                MockGps(clock, profile, mission_time),
                MockBattery(clock, profile, mission_time))
    if source is None:
        raise ValueError("FLIGHT/HIL profili MAVLink kaynağı gerektirir (source=None)")
    return (MavlinkBarometer(source), MavlinkImu(source),
            MavlinkGps(source), MavlinkBattery(source))


def create_actuators(config: AppConfig, log=print):
    """
    Aktüatör suite'i döndürür. SIMULATION_ONLY → mock ActuatorSuite (fiziksel PWM
    yok, yalnız komut logu); FLIGHT/HIL → RealActuatorSuite (ayrılma CH14/CH13 ve
    kanat CH15 GERÇEK PCA9685'ten sürülür). Böylece 'AYIR' (Manuel Ayrılma) komutu
    ve otonom ayrılma gerçek servoları döndürür. Motor/APAM/buzzer her iki profilde
    de mock kalır (MAVLink/ApamActuator üzerinden ayrı sürülür).

    FLIGHT/HIL'de PCA9685 open() ana döngüde çağrılır (donanım yoksa açık hata
    loglanır, suite set_us no-op ile yaşamaya devam eder — güvenli degrade).
    """
    if config.profile is RunProfile.SIMULATION_ONLY:
        from src.drivers.mock_actuators import ActuatorSuite
        return ActuatorSuite()
    from src.drivers.real_actuators import RealActuatorSuite
    return RealActuatorSuite(log=log)


def create_flight_controller(config: AppConfig, clock, source=None, attitude_fn=None):
    """
    Uçuş kontrol kartı bağı döndürür. SIMULATION_ONLY → SimulatedFlightControllerLink
    (attitude_fn kestiriciden okur); FLIGHT/HIL → MavlinkFlightControllerLink (Mini Pix).
    """
    if config.profile is RunProfile.SIMULATION_ONLY:
        return SimulatedFlightControllerLink(config.control, clock, attitude_fn=attitude_fn)
    if source is None:
        raise ValueError("FLIGHT/HIL profili MAVLink kaynağı gerektirir (source=None)")
    return MavlinkFlightControllerLink(source, clock)
