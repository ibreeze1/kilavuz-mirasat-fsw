"""APAM / failsafe testleri — Şartname G-10 (REQ-SAFE-003..008, CONFLICT önlemleri)."""
from config.default import ApamConfig
from src.drivers.mock_actuators import ActuatorSuite
from src.hal.interfaces import ArmState, ServoPosition
from src.mission.context import FlightContext
from src.services.failsafe import ApamTrigger, FailsafeManager


def _ctx(t, **kw):
    base = dict(mission_time_s=t, altitude_m=500.0, descent_speed_mps=20.0,
                ascending=False, gps_valid=True, separation_confirmed=True)
    base.update(kw)
    return FlightContext(**base)


def _step(fm, start, stop, step, **kw):
    """Görev zamanını start→stop arası ilerletip her adımda update çağırır."""
    dec = None
    t = start
    while t <= stop + 1e-9:
        dec = fm.update(_ctx(t, **kw))
        t += step
    return dec


def test_triggers_after_16mps_10s():
    fm = FailsafeManager(ApamConfig())
    fm.update(_ctx(0.0))                       # sayaç referansı
    dec = _step(fm, 1.0, 11.0, 1.0)            # 20 m/s, 10 s+ kesintisiz
    assert dec.apam_active
    assert dec.trigger is ApamTrigger.ALGORITHM


def test_no_trigger_before_10s():
    fm = FailsafeManager(ApamConfig())
    fm.update(_ctx(0.0))
    dec = _step(fm, 1.0, 8.0, 1.0)             # yalnız 8 s
    assert not dec.apam_active


def test_counter_resets_when_speed_safe():
    """Hız güvenli seviyeye düşerse sayaç sıfırlanır (anlık artış tetiklemez)."""
    fm = FailsafeManager(ApamConfig())
    fm.update(_ctx(0.0))
    _step(fm, 1.0, 7.0, 1.0, descent_speed_mps=20.0)   # 7 s aşırı hız
    # hız güvene döner → sıfırla
    dec = fm.update(_ctx(8.0, descent_speed_mps=9.0))
    assert dec.overspeed_timer_s == 0.0
    # tekrar aşırı hız ama yalnız 5 s → tetiklenmemeli
    dec = _step(fm, 9.0, 13.0, 1.0, descent_speed_mps=20.0)
    assert not dec.apam_active


def test_altitude_guard_below_100m_no_deploy():
    """İrtifa ≤100 m ise paraşüt açılmaz (kesin >100 m kuralı)."""
    fm = FailsafeManager(ApamConfig())
    fm.update(_ctx(0.0, altitude_m=80.0))
    dec = _step(fm, 1.0, 12.0, 1.0, altitude_m=80.0)
    assert not dec.apam_active


def test_no_trigger_during_ascent():
    """Yükselme sırasında APAM tetiklenmez (iniş fazı kilidi)."""
    fm = FailsafeManager(ApamConfig())
    fm.update(_ctx(0.0, ascending=True, separation_confirmed=False))
    dec = _step(fm, 1.0, 15.0, 1.0, ascending=True, separation_confirmed=False,
                descent_speed_mps=20.0)
    assert not dec.apam_active


def test_manual_apam_triggers_above_100m():
    fm = FailsafeManager(ApamConfig())
    dec = fm.update(_ctx(0.0, descent_speed_mps=9.0, manual_apam_cmd=True,
                         altitude_m=300.0))
    assert dec.apam_active and dec.trigger is ApamTrigger.MANUAL


def test_manual_apam_deploys_below_100m():
    """Manuel APAM irtifadan BAĞIMSIZ açılır (operatör kararı; QR tezgah demosu
    için de gerekli). İrtifa sınırı yalnız ALGORİTMİK tetikte geçerlidir."""
    fm = FailsafeManager(ApamConfig())
    dec = fm.update(_ctx(0.0, descent_speed_mps=9.0, manual_apam_cmd=True,
                         altitude_m=0.0))
    assert dec.apam_active and dec.trigger is ApamTrigger.MANUAL


def test_link_loss_alone_does_not_trigger():
    """Link loss TEK BAŞINA APAM tetiklemez (hız güvenli)."""
    fm = FailsafeManager(ApamConfig())
    fm.update(_ctx(0.0, descent_speed_mps=9.0))
    from src.mission.context import HealthFlags
    dec = _step(fm, 1.0, 20.0, 1.0, descent_speed_mps=9.0,
                health=HealthFlags(link_loss=True))
    assert not dec.apam_active


def test_execute_apam_kills_motors_before_parachute():
    """KESİN sıra: önce motor kill, sonra paraşüt OPEN."""
    fm = FailsafeManager(ApamConfig())
    suite = ActuatorSuite()
    suite.motors.arm().unwrap()
    suite.motors.set_throttle(0.7).unwrap()
    r = fm.execute_apam(suite)
    assert r.is_ok
    assert suite.motors.throttle == 0.0
    assert suite.motors.arm_state is ArmState.DISARMED
    assert suite.apam_servo.position is ServoPosition.OPEN
    # komut logunda KILL, MOVE OPEN'dan ÖNCE gelmeli
    kill_i = suite.motors.command_log.index("KILL")
    assert "KILL" in suite.motors.command_log
    assert suite.apam_servo.command_log[-1] == "MOVE OPEN"
    assert kill_i >= 0
    assert fm.parachute_deployed


def test_contradicting_gps_blocks_apam():
    """GPS mevcut ve baro aşırı hızını yalanlıyorsa APAM tetiklenmez (false-trigger)."""
    fm = FailsafeManager(ApamConfig())
    fm.update(_ctx(0.0, gps_valid=True, speed_consistent=False))
    dec = _step(fm, 1.0, 15.0, 1.0, gps_valid=True, speed_consistent=False)
    assert not dec.apam_active


def test_gps_lost_still_triggers_single_sensor():
    """GPS yoksa çoklu-sensör filtresi uygulanmaz; APAM tek sensörle tetiklenir."""
    fm = FailsafeManager(ApamConfig())
    fm.update(_ctx(0.0, gps_valid=False, speed_consistent=False))
    dec = _step(fm, 1.0, 12.0, 1.0, gps_valid=False, speed_consistent=False)
    assert dec.apam_active


def test_apam_latches_active():
    fm = FailsafeManager(ApamConfig())
    fm.update(_ctx(0.0))
    _step(fm, 1.0, 11.0, 1.0)
    # hız güvene dönse bile APAM aktif kalır (latch)
    dec = fm.update(_ctx(12.0, descent_speed_mps=5.0))
    assert dec.apam_active
