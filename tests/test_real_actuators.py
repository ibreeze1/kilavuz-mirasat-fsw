"""
Gerçek PCA9685 aktüatör suite'i testleri (Gereksinim-7 — Manuel Ayrılma gerçek servo).
Donanımsız (bus=None) çalışır: set_us no-op'tur, yalnız MANTIK doğrulanır. Fiziksel
PWM doğrulaması RPi'de tools/separation_bench_test.py --separate ile yapılır.
"""
from config.default import AppConfig, RunProfile, get_config
from src.drivers.factory import create_actuators
from src.drivers.mock_actuators import ActuatorSuite
from src.drivers.real_actuators import RealActuatorSuite
from src.hal.interfaces import ServoPosition

_NOLOG = lambda _s: None   # noqa: E731  test çıktısını sessizleştir


# ------------------------------------------------------- fabrika profil dallanması
def test_simulation_actuators_are_mock():
    suite = create_actuators(get_config(), _NOLOG)      # SIMULATION_ONLY
    assert isinstance(suite, ActuatorSuite)
    assert not isinstance(suite, RealActuatorSuite)


def test_flight_actuators_are_real():
    cfg = AppConfig(profile=RunProfile.FLIGHT)
    suite = create_actuators(cfg, _NOLOG)
    assert isinstance(suite, RealActuatorSuite)


# --------------------------------------------------------- ayrılma mekanizması mantığı
def test_real_separation_starts_locked():
    suite = RealActuatorSuite(log=_NOLOG)
    assert suite.separation.locked                      # iki servo da LOCKED
    assert not suite.separation.released


def test_real_separation_release_opens_both():
    suite = RealActuatorSuite(log=_NOLOG)
    r = suite.separation.release()
    assert r.is_ok
    assert suite.separation.released                    # komut-tabanlı geri bildirim
    assert suite.separation.left.position is ServoPosition.OPEN
    assert suite.separation.right.position is ServoPosition.OPEN
    assert not suite.separation.locked


def test_real_separation_to_safe_relocks():
    suite = RealActuatorSuite(log=_NOLOG)
    suite.separation.release()
    suite.separation.to_safe()
    assert suite.separation.locked
    # Ayrılma FİZİKSEL/geri dönüşsüz → released bayrağı korunur (kol semantiği).
    assert suite.separation.released


class _RecPca:
    """set_us çağrılarını kaydeden sahte PCA9685 (fiziksel PWM yazımını gözler)."""

    def __init__(self) -> None:
        self.writes: list[tuple] = []

    def set_us(self, channel: int, pulse_us: float) -> None:
        self.writes.append((channel, pulse_us))


def test_separation_to_safe_after_release_does_not_drive_servos_back():
    """BUG düzeltmesi: ayrıldıktan sonra to_safe() servoları LOCKED'a GERİ SÜRMEZ
    (yoksa kapanışta servolar kendi kendine kilitli konuma döner)."""
    from src.drivers.real_actuators import RealSeparationMechanism
    pca = _RecPca()
    sep = RealSeparationMechanism(pca)
    sep.release()
    pca.writes.clear()
    sep.to_safe()
    assert pca.writes == []               # released sonrası HİÇ PWM yazılmaz
    assert sep.released


def test_separation_to_safe_before_release_locks_servos():
    """Henüz ayrılmadıysa to_safe() güvenli-kilit PWM'i yazar (kurulum/güvenlik)."""
    from src.drivers.real_actuators import CH_SEP_LEFT, CH_SEP_RIGHT, RealSeparationMechanism
    pca = _RecPca()
    sep = RealSeparationMechanism(pca)
    sep.to_safe()
    channels = {w[0] for w in pca.writes}
    assert channels == {CH_SEP_LEFT, CH_SEP_RIGHT}   # iki servo da LOCKED'a


# ------------------------------------------------------------- kanat mekanizması mantığı
def test_real_arms_deploy_and_lock():
    suite = RealActuatorSuite(log=_NOLOG)
    assert suite.arms.locked and not suite.arms.deployed
    r = suite.arms.deploy_and_lock()
    assert r.is_ok
    assert suite.arms.deployed and suite.arms.locked


def test_real_arms_to_safe_after_deploy_keeps_deployed():
    suite = RealActuatorSuite(log=_NOLOG)
    suite.arms.deploy_and_lock()
    suite.arms.to_safe()
    assert suite.arms.deployed                          # geri katlamaz (güvenlik)
    assert suite.arms.locked


# --------------------------------------------------------------- APAM paraşüt servosu
def test_real_apam_servo_starts_closed():
    from src.drivers.real_actuators import RealApamServo
    suite = RealActuatorSuite(log=_NOLOG)
    assert isinstance(suite.apam_servo, RealApamServo)   # mock değil, gerçek
    assert suite.apam_servo.position is ServoPosition.CLOSED


def test_real_apam_deploy_opens_and_execute_apam_wires_through():
    from src.services.failsafe import FailsafeManager
    from config.default import ApamConfig
    suite = RealActuatorSuite(log=_NOLOG)
    # Doğrudan servo: OPEN → paraşüt bırak.
    assert suite.apam_servo.move_to(ServoPosition.OPEN).is_ok
    assert suite.apam_servo.position is ServoPosition.OPEN
    # failsafe.execute_apam gerçek servoyu sürer (motor kill → paraşüt OPEN).
    suite.apam_servo.to_safe()                           # CLOSED'a geri al
    r = FailsafeManager(ApamConfig()).execute_apam(suite)
    assert r.is_ok
    assert suite.apam_servo.position is ServoPosition.OPEN
    assert suite.motors.throttle == 0.0                  # önce motor kill (G-10)


def test_real_apam_safe_state_closes_parachute():
    suite = RealActuatorSuite(log=_NOLOG)
    suite.apam_servo.move_to(ServoPosition.OPEN)
    suite.enter_safe_state()                             # APAM YANLIŞLIKLA açık kalmaz
    assert suite.apam_servo.position is ServoPosition.CLOSED


# ------------------------------------------------------------- donanımsız güvenli degrade
def test_real_suite_no_hardware_does_not_raise():
    suite = RealActuatorSuite(log=_NOLOG)                # bus yok → set_us no-op
    # Tüm hareket komutları donanımsız da exception atmamalı.
    suite.enter_safe_state()
    suite.separation.release()
    suite.arms.deploy_and_lock()
    suite.enter_safe_state()
    open_result = suite.open()                          # smbus2/çip yok → açık hata
    assert open_result.is_err                           # çökme yok, Result.err
    suite.close()
