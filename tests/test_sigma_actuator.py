"""
SİGMA motor yer-testi aktüatörü testleri — 'SIGMA' tetiğinde Mini Pix'e
MAV_CMD_DO_MOTOR_TEST gönderimi. Gerçek donanım/pymavlink OLMADAN, sahte (fake)
mavlink bağlantısıyla (QR tezgah demosu, GPS'siz).
"""
from dataclasses import replace

from config.default import AppConfig, RunProfile, with_overrides
from src.common.result import ErrorCode
from src.drivers.sigma_actuator import (
    MAV_CMD_COMPONENT_ARM_DISARM,
    MAV_CMD_DO_MOTOR_TEST,
    MOTOR_COUNT,
    MOTOR_TEST_ORDER_SEQUENCE,
    MOTOR_TEST_THROTTLE_PERCENT,
    SAFE_MAX_PERCENT,
    SigmaMotorActuator,
)


class _FakeMav:
    def __init__(self) -> None:
        self.sent: list[tuple] = []

    def command_long_send(self, target_system, target_component, command,
                          confirmation, p1, p2, p3, p4, p5, p6, p7) -> None:
        self.sent.append((command, p1, p2, p3, p4, p5, p6, p7))


class _FakeAck:
    def __init__(self, result: int) -> None:
        self.result = result


class FakeConn:
    def __init__(self, ack_result: int | None = 0) -> None:
        self.mav = _FakeMav()
        self.target_system = 1
        self.target_component = 1
        self._ack = ack_result

    def recv_match(self, type=None, blocking=False, timeout=None):
        return None if self._ack is None else _FakeAck(self._ack)


def _flight_config() -> AppConfig:
    return AppConfig(profile=RunProfile.FLIGHT)


def _sim_config() -> AppConfig:
    return AppConfig()


# --------------------------------------------------------------- gerçek yol (FLIGHT)
def test_trigger_sends_single_do_motor_test():
    conn = FakeConn()
    act = SigmaMotorActuator(_flight_config(), connect_fn=lambda p, b: conn)
    res = act.trigger()

    assert res.is_ok
    assert len(conn.mav.sent) == 1                    # TEK komut (FC sekansı yürütür)
    command, p1, p2, p3, p4, p5, p6, p7 = conn.mav.sent[0]
    assert command == MAV_CMD_DO_MOTOR_TEST
    assert p1 == 1.0                                  # başlangıç motoru
    assert p2 == float(MOTOR_TEST_THROTTLE_PERCENT)   # gaz tipi = yüzde
    assert p5 == float(MOTOR_COUNT)                   # 4 motoru sırayla
    assert p6 == float(MOTOR_TEST_ORDER_SEQUENCE)     # sıra numarasına göre
    assert act.fire_count == 1


def test_percent_clamped_to_safe_max():
    base = _flight_config()
    # sigma_test_percent'i güvenli üst sınırın çok üstüne çek → clamp beklenir.
    cfg = with_overrides(base, control=replace(base.control, sigma_test_percent=90.0))
    conn = FakeConn()
    act = SigmaMotorActuator(cfg, connect_fn=lambda p, b: conn)
    act.trigger()
    _, _p1, _p2, p3, *_ = conn.mav.sent[0]
    assert p3 == SAFE_MAX_PERCENT


def test_retriggerable_counts_up():
    conn = FakeConn()
    act = SigmaMotorActuator(_flight_config(), connect_fn=lambda p, b: conn)
    act.trigger()
    act.trigger()
    act.trigger()
    assert act.fire_count == 3
    assert len(conn.mav.sent) == 3                    # her tetik ayrı komut


def test_connection_failure_is_explicit_not_crash():
    def _boom(port, baud):
        raise RuntimeError("port yok")

    logs: list[str] = []
    act = SigmaMotorActuator(_flight_config(), log=logs.append, connect_fn=_boom)
    res = act.trigger()
    assert res.is_err and res.code is ErrorCode.IO_ERROR
    assert act.fire_count == 1                         # denendi (sayaç arttı)


def test_ack_not_yet_still_ok():
    conn = FakeConn(ack_result=None)                  # ACK henüz gelmedi
    logs: list[str] = []
    act = SigmaMotorActuator(_flight_config(), log=logs.append,
                             connect_fn=lambda p, b: conn)
    res = act.trigger()
    assert res.is_ok                                  # komut gitti; ACK best-effort
    assert len(conn.mav.sent) == 1


# ------------------------------------------------------------- motor STOP (APAM/G-10)
def test_stop_zeroes_all_motors_and_disarms():
    conn = FakeConn()
    act = SigmaMotorActuator(_flight_config(), connect_fn=lambda p, b: conn)
    assert act.stop().is_ok
    motor_tests = [s for s in conn.mav.sent if s[0] == MAV_CMD_DO_MOTOR_TEST]
    disarms = [s for s in conn.mav.sent if s[0] == MAV_CMD_COMPONENT_ARM_DISARM]
    assert len(motor_tests) == MOTOR_COUNT
    assert all(s[3] == 0.0 for s in motor_tests)      # gaz %0
    assert len(disarms) == 1 and disarms[0][1] == 0.0  # DISARM (param1=0)


def test_stop_in_simulation_sends_nothing():
    conn = FakeConn()
    act = SigmaMotorActuator(_sim_config(), connect_fn=lambda p, b: conn)
    assert act.stop().is_ok
    assert conn.mav.sent == []


# ---------------------------------------------------------------------- sim yolu
def test_simulation_does_not_send_real_command():
    conn = FakeConn()
    logs: list[str] = []
    act = SigmaMotorActuator(_sim_config(), log=logs.append,
                             connect_fn=lambda p, b: conn)
    res = act.trigger()
    assert res.is_ok
    assert conn.mav.sent == []                        # donanıma dokunulmadı
    assert any("SIM" in line for line in logs)
