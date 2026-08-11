from pymavlink import mavutil
import time

GAZ = 20
SURE = 2

m = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
m.wait_heartbeat(timeout=10)

print(f"4 motor SIRAYLA, %{GAZ} gaz, her biri {SURE} saniye")
print("Guvenlik anahtari BASILI ve LED SABIT olmali\n")
for i in (3, 2, 1):
    print(f"  {i}")
    time.sleep(1)

for motor in (1, 2, 3, 4):
    print(f"Motor {motor}...")
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST, 0,
        motor, 0, GAZ, SURE, 0, 0, 0)
    time.sleep(SURE + 1)

print("bitti")
