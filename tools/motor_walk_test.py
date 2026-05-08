#!/usr/bin/env python3
import serial
import time

PORT = "/dev/ttyACM0"
BAUD = 115200

ROWS = 5
COLS = 10
N = ROWS * COLS

ON_VALUE = 100

SEND_HZ = 100            # frecuencia de envio (frames por segundo)
REPEAT_PER_MOTOR = 40   # cuantos frames repetir antes de pasar al siguiente motor

dt = 1.0 / SEND_HZ

def build_frame(active_idx):
    vals = [0]*N
    if 0 <= active_idx < N:
        vals[active_idx] = ON_VALUE
    return f"G {ROWS} {COLS} " + " ".join(map(str, vals)) + "\n"


print("abriendo serial...")
ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)

try:
    while True:

        for motor in range(N):

            frame = build_frame(motor).encode()

            for _ in range(REPEAT_PER_MOTOR):

                ser.write(frame)
                ser.flush()
                time.sleep(dt)

            print("motor", motor)

except KeyboardInterrupt:
    pass

finally:
    off = build_frame(-1).encode()
    ser.write(off)
    ser.close()
    print("apagado")

