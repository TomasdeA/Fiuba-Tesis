#!/usr/bin/env python3
"""
motor_column_sweep.py
---------------------
Prueba de comunicación UART con el ESP32.
Sin dependencias externas — usa solo stdlib (termios/fcntl).

Recorre las 10 columnas de la grilla 5x10 de derecha a izquierda
(col 9 → col 0, igual que el índice de la trama M).
En cada paso enciende al 100 una columna entera y apaga el resto.

Protocolo: "M d0 d1 ... d49\n"
  índice: (9 - col) * 5 + row   →  col=9,row=0 → d0 (sup-der)
                                    col=0,row=4 → d49 (inf-izq)

Uso:
    python3 motor_column_sweep.py [--port /dev/serial0] [--baud 115200] [--delay 0.8] [--duty 100] [--repeat 1]
"""

import argparse
import fcntl
import glob
import os
import termios
import time

ROWS = 5
COLS = 10
N    = ROWS * COLS  # 50

BAUD_MAP = {
    9600:   termios.B9600,
    19200:  termios.B19200,
    38400:  termios.B38400,
    57600:  termios.B57600,
    115200: termios.B115200,
    230400: termios.B230400,
}


def serial_candidates(port_hint: str) -> list:
    candidates = []
    if port_hint:
        candidates.append(port_hint)
    for base in ("/dev/serial0", "/dev/ttyAMA0", "/dev/ttyS0"):
        if base not in candidates:
            candidates.append(base)
    for base in ("/dev/ttyACM0", "/dev/ttyACM1"):
        if base not in candidates:
            candidates.append(base)
    for path in sorted(glob.glob("/dev/ttyACM*")):
        if path not in candidates:
            candidates.append(path)
    return candidates


def open_serial(port: str, baud: int) -> tuple[int, str]:
    last_error = None
    for candidate in serial_candidates(port):
        try:
            fd = os.open(candidate, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        except OSError as exc:
            last_error = exc
            continue

        # Quitar O_NONBLOCK para escritura bloqueante
        flags = fcntl.fcntl(fd, fcntl.F_GETFL)
        fcntl.fcntl(fd, fcntl.F_SETFL, flags & ~os.O_NONBLOCK)

        attrs = termios.tcgetattr(fd)
        # cflag: 8N1, sin flow control
        attrs[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
        # iflag, oflag, lflag: raw
        attrs[0] = 0
        attrs[1] = 0
        attrs[3] = 0
        # cc
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 1
        # velocidad
        spd = BAUD_MAP.get(baud, termios.B115200)
        attrs[4] = spd  # ispeed
        attrs[5] = spd  # ospeed
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        return fd, candidate

    if last_error is None:
        raise OSError("No se encontraron puertos /dev/ttyACM*")
    raise last_error


def build_frame(duties: list) -> bytes:
    return ("M " + " ".join(str(d) for d in duties) + "\n").encode("ascii")


def col_frame(active_col: int, duty: int) -> bytes:
    duties = [0] * N
    for row in range(ROWS):
        idx = (COLS - 1 - active_col) * ROWS + row
        duties[idx] = duty
    return build_frame(duties)


def all_off_frame() -> bytes:
    return build_frame([0] * N)


def main():
    parser = argparse.ArgumentParser(description="Barrido de columnas para ESP32")
    parser.add_argument("--port",   default="/dev/serial0")
    parser.add_argument("--baud",   type=int,   default=115200)
    parser.add_argument("--delay",  type=float, default=0.8, help="Segundos por columna")
    parser.add_argument("--duty",   type=int,   default=100, help="Duty cycle activo (0-100)")
    parser.add_argument("--repeat", type=int,   default=1,   help="Repeticiones del barrido")
    args = parser.parse_args()

    print(f"Abriendo UART ({args.port}) @ {args.baud} bps")
    fd, connected_port = open_serial(args.port, args.baud)
    print(f"Conectado a {connected_port}")
    try:
        time.sleep(0.1)  # esperar posible reset del ESP32

        for rep in range(args.repeat):
            print(f"\n── Barrido {rep + 1}/{args.repeat} ──")
            for col in range(COLS - 1, -1, -1):  # col 9 → col 0
                frame = col_frame(col, args.duty)
                os.write(fd, frame)
                print(f"  col {col:2d}  →  {frame.decode().strip()}")
                time.sleep(args.delay)

        os.write(fd, all_off_frame())
        print("\nTodos los motores apagados. Listo.")
    finally:
        os.close(fd)


if __name__ == "__main__":
    main()
