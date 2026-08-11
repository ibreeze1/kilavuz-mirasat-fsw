import smbus2, time

ADDR = 0x40
MODE1, PRESCALE, LED0_ON_L = 0x00, 0xFE, 0x06

bus = smbus2.SMBus(1)

# 50 Hz servo frekansi
bus.write_byte_data(ADDR, MODE1, 0x10)          # uyku
bus.write_byte_data(ADDR, PRESCALE, 121)        # 25MHz/(4096*50)-1
bus.write_byte_data(ADDR, MODE1, 0x00)          # uyan
time.sleep(0.005)
bus.write_byte_data(ADDR, MODE1, 0xA0)          # restart + auto-increment

def pwm_us(kanal, us):
    tik = int(us * 4096 / 20000)                # 20 ms periyot
    reg = LED0_ON_L + 4 * kanal
    bus.write_byte_data(ADDR, reg + 0, 0)
    bus.write_byte_data(ADDR, reg + 1, 0)
    bus.write_byte_data(ADDR, reg + 2, tik & 0xFF)
    bus.write_byte_data(ADDR, reg + 3, tik >> 8)
    print(f"  CH{kanal} -> {us} us ({tik} tik)")

print("KILITLI konum (1000 us)")
for ch in (13, 14):
    pwm_us(ch, 1000)
time.sleep(3)

print("ACIK konum (2000 us)")
for ch in (13, 14):
    pwm_us(ch, 2000)
time.sleep(3)

print("KILITLI konuma donuluyor")
for ch in (13, 14):
    pwm_us(ch, 1000)
