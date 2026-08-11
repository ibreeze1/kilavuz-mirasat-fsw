"""
SİGMA itki motorları tezgah bring-up testi (tools/motor_bench_test.py) birim testleri.
Gerçek donanım/pymavlink OLMADAN, sahte (fake) mavlink bağlantısıyla: heartbeat,
DO_MOTOR_TEST komut/parametre eşlemesi, güvenli gaz clamp'i, --all sıralaması, stop
(disarm), ACK teyidi ve bağlantı hatasında sessiz çökme YOK.
"""
from tools.motor_bench_test import (
    MAV_CMD_COMPONENT_ARM_DISARM,
    MAV_CMD_DO_MOTOR_TEST,
    MOTOR_COUNT,
    MOTOR_TEST_THROTTLE_PERCENT,
    MAV_RESULT_ACCEPTED,
    SAFE_MAX_PERCENT,
    MotorTester,
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
    def __init__(self, ack_result: int | None = MAV_RESULT_ACCEPTED,
                 heartbeat: bool = True) -> None:
        self.mav = _FakeMav()
        self.target_system = 1
        self.target_component = 1
        self._ack = ack_result
        self._hb = heartbeat
        self.closed = False

    def wait_heartbeat(self, timeout=None):
        return object() if self._hb else None

    def recv_match(self, type=None, blocking=False, timeout=None):
        return None if self._ack is None else _FakeAck(self._ack)

    def close(self):
        self.closed = True


def _tester(conn, **kw) -> MotorTester:
    return MotorTester(connect_fn=lambda port, baud: conn, log=lambda s: None, **kw)


# ------------------------------------------------------------------- bağlantı
def test_open_requires_heartbeat():
    ok_conn = FakeConn(heartbeat=True)
    assert _tester(ok_conn).open() is True

    no_hb = FakeConn(heartbeat=False)
    t = _tester(no_hb)
    assert t.open() is False
    assert "heartbeat" in (t.error or "").lower()


def test_connection_failure_is_explicit_not_crash():
    def _boom(port, baud):
        raise RuntimeError("port yok")

    t = MotorTester(connect_fn=_boom, log=lambda s: None)
    assert t.open() is False
    assert "açılamadı" in (t.error or "")


# ------------------------------------------------------------- tek motor komutu
def test_test_motor_sends_correct_do_motor_test():
    conn = FakeConn()
    t = _tester(conn)
    t.open()
    assert t.test_motor(3, percent=5, seconds=2) is True

    assert len(conn.mav.sent) == 1
    command, p1, p2, p3, p4, p5, p6, p7 = conn.mav.sent[0]
    assert command == MAV_CMD_DO_MOTOR_TEST
    assert p1 == 3.0                                  # motor no
    assert p2 == float(MOTOR_TEST_THROTTLE_PERCENT)   # gaz tipi = yüzde
    assert p3 == 5.0                                  # gaz %
    assert p4 == 2.0                                  # süre
    assert p5 == 1.0                                  # tek motor


def test_percent_clamped_to_safe_max():
    conn = FakeConn()
    t = _tester(conn)
    t.open()
    t.test_motor(1, percent=90, seconds=1)            # güvenli üst sınırın çok üstü
    _, _p1, _p2, p3, *_ = conn.mav.sent[0]
    assert p3 == SAFE_MAX_PERCENT                      # %90 -> clamp


# ------------------------------------------------------------------ --all / stop
def test_test_all_sequences_every_motor():
    conn = FakeConn()
    t = _tester(conn)
    t.open()
    ok = t.test_all(percent=5, seconds=0)             # seconds=0 → dry değil ama bekleme yok
    assert ok == MOTOR_COUNT
    motors = [s[1] for s in conn.mav.sent]
    assert motors == [float(m) for m in range(1, MOTOR_COUNT + 1)]


def test_stop_zeroes_all_then_disarms():
    conn = FakeConn()
    t = _tester(conn)
    t.open()
    assert t.stop() is True
    # MOTOR_COUNT adet %0 DO_MOTOR_TEST + 1 DISARM
    motor_tests = [s for s in conn.mav.sent if s[0] == MAV_CMD_DO_MOTOR_TEST]
    disarms = [s for s in conn.mav.sent if s[0] == MAV_CMD_COMPONENT_ARM_DISARM]
    assert len(motor_tests) == MOTOR_COUNT
    assert all(s[3] == 0.0 for s in motor_tests)      # gaz %0
    assert len(disarms) == 1
    assert disarms[0][1] == 0.0                        # param1=0 → DISARM


# --------------------------------------------------------------------- ACK yolu
def test_ack_rejected_returns_false():
    conn = FakeConn(ack_result=1)                     # != ACCEPTED
    t = _tester(conn)
    t.open()
    assert t.test_motor(1, percent=5, seconds=1) is False


def test_ack_timeout_returns_false():
    conn = FakeConn(ack_result=None)                  # ACK hiç gelmez
    t = _tester(conn)
    t.open()
    assert t.test_motor(1, percent=5, seconds=1) is False


# ------------------------------------------------------------------- dry-run yolu
def test_dry_run_sends_nothing():
    conn = FakeConn()
    t = _tester(conn, dry_run=True)
    assert t.open() is True
    t.test_motor(1, percent=5, seconds=1)
    t.test_all(percent=5, seconds=1)
    t.stop()
    assert conn.mav.sent == []                         # donanıma hiç dokunulmadı
