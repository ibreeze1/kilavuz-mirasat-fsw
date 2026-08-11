from pymavlink import mavutil
import sys, time

motor = int(sys.argv[1]) if len(sys.argv) > 1 else 1
gaz   = int(sys.argv[2]) if len(sys.argv) > 2 else 5
sure  = 3

m = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
m.wait_heartbeat(timeout=10)

print(f"Motor {motor}, %{gaz} gaz, {sure} saniye")
print("Guvenlik anahtari BASILI ve LED SABIT olmali\n")

m.mav.command_long_send(
    m.target_system, m.target_component,
    mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST, 0,
    motor,   # 1-4
    0,       # 0 = yuzde
    gaz,
    sure,
    0, 0, 0)

t0 = time.time()
while time.time() - t0 < 3:
    msg = m.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
    if msg and msg.command == mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST:
        sonuc = "KABUL" if msg.result == 0 else f"RED (result={msg.result})"
        print("Ucus karti:", sonuc)
        break
else:
    print("ACK gelmedi")

time.sleep(sure + 1)
print("bitti")
