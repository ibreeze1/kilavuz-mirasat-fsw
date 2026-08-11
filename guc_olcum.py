from pymavlink import mavutil
import time

SURE = 4
KADEMELER = [10, 20, 30, 40, 50]

m = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
m.wait_heartbeat(timeout=10)
m.mav.request_data_stream_send(m.target_system, m.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)

def olc(saniye):
    """Belirtilen sure boyunca gerilim/akim orneklerinin ortalamasini alir."""
    v, a, n = 0.0, 0.0, 0
    t0 = time.time()
    while time.time() - t0 < saniye:
        msg = m.recv_match(type='SYS_STATUS', blocking=True, timeout=1)
        if msg:
            v += msg.voltage_battery / 1000.0
            a += msg.current_battery / 100.0
            n += 1
    return (v / n, a / n) if n else (0.0, 0.0)

print("Bosta olculuyor...")
v0, a0 = olc(3)
print(f"  {v0:.2f} V   {a0:.2f} A   {v0 * a0:.1f} W\n")

print(f"{'Gaz':>5} {'Gerilim':>9} {'Akim':>8} {'Guc':>9}  (tek motor)")
print("-" * 45)

for gaz in KADEMELER:
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST, 0,
        1, 0, gaz, SURE, 0, 0, 0)
    time.sleep(1.0)                 # motorun hizlanmasi
    v, a = olc(SURE - 1.5)
    print(f"{gaz:>4}% {v:>8.2f}V {a:>7.2f}A {v*a:>8.1f}W")
    time.sleep(2)                   # soguma

print("\nNOT: degerler TEK motor icin. Dort motor kabaca 4 katidir,")
print("ama pil gerilimi dustukce akim artar - gercek ucus daha yuksek olur.")
