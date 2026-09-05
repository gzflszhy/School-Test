# main.py — MaixCam Pro → C板 USART6 AVC1 最小测试

import struct
from maix import err, pinmap, time, uart

# 关键：先把 MaixCam Pro 排针 A16 复用为 UART1_TX。
err.check_raise(pinmap.set_pin_function("A16", "UART1_TX"), "set TX pin failed")

print(uart.list_devices())  # 应包含 /dev/ttyS1

MAGIC = 0x31435641
SEED = 0xA5A51234


def cksum(data: bytes) -> int:
    c = SEED
    for b in data:
        c = ((c << 5) ^ (c >> 2) ^ b) & 0xFFFFFFFF
    return c


def pack_avc1(seq, vx, vy, valid):
    payload = struct.pack("<IIffB3x", MAGIC, seq, vx, vy, 1 if valid else 0)
    return payload + struct.pack("<I", cksum(payload))


serial = uart.UART("/dev/ttyS1", 115200)  # MaixCam UART 排针
seq = 0
t0 = time.time()

while True:
    phase = (time.time() - t0) % 15.0
    vx = 0.05 if 5.0 <= phase < 10.0 else 0.0
    n = serial.write(pack_avc1(seq, vx, 0.0, True))
    print("sent", n)  # 正常应恒为 24
    seq = (seq + 1) & 0xFFFFFFFF
    time.sleep_ms(50)  # 20 Hz
