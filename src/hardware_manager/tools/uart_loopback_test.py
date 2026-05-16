#!/usr/bin/env python3
"""
uart_loopback_test.py
---------------------
Prueba de loopback UART (puentear TX y RX) para validar escritura/lectura.

Usa la misma configuracion de puerto que motor_column_sweep.py:
- busqueda de candidatos
- termios raw 8N1
- baudrate configurable

Uso:
  python3 src/hardware_manager/tools/uart_loopback_test.py --port /dev/serial0
"""

import argparse
import fcntl
import glob
import os
import termios
import time

BAUD_MAP = {
    9600: termios.B9600,
    19200: termios.B19200,
    38400: termios.B38400,
    57600: termios.B57600,
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

        flags = fcntl.fcntl(fd, fcntl.F_GETFL)
        fcntl.fcntl(fd, fcntl.F_SETFL, flags & ~os.O_NONBLOCK)

        attrs = termios.tcgetattr(fd)
        attrs[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
        attrs[0] = 0
        attrs[1] = 0
        attrs[3] = 0
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 1
        spd = BAUD_MAP.get(baud, termios.B115200)
        attrs[4] = spd
        attrs[5] = spd
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        return fd, candidate

    if last_error is None:
        raise OSError("No se encontraron puertos UART candidatos")
    raise last_error


def read_with_timeout(fd: int, timeout_s: float) -> bytes:
    deadline = time.monotonic() + timeout_s
    chunks = []
    while time.monotonic() < deadline:
        try:
            data = os.read(fd, 4096)
        except BlockingIOError:
            data = b""
        if data:
            chunks.append(data)
        else:
            time.sleep(0.01)
    return b"".join(chunks)


def main() -> int:
    parser = argparse.ArgumentParser(description="Prueba UART loopback RX<->TX")
    parser.add_argument("--port", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--msg", default="UART_LOOPBACK_TEST_12345")
    parser.add_argument("--repeat", type=int, default=5)
    parser.add_argument("--timeout", type=float, default=0.35, help="Timeout de lectura por intento")
    args = parser.parse_args()

    print(f"Abriendo UART ({args.port}) @ {args.baud} bps")
    fd, connected_port = open_serial(args.port, args.baud)
    print(f"Conectado a {connected_port}")

    try:
        time.sleep(0.1)
        ok = 0
        fail = 0

        for i in range(1, args.repeat + 1):
            payload = f"{args.msg}__{i}\n".encode("ascii", errors="replace")
            os.write(fd, payload)
            rx = read_with_timeout(fd, args.timeout)
            matched = payload in rx

            if matched:
                ok += 1
                print(f"[OK  ] intento {i}: TX={payload!r} RX={rx!r}")
            else:
                fail += 1
                print(f"[FAIL] intento {i}: TX={payload!r} RX={rx!r}")

        print(f"\nResultado: OK={ok} FAIL={fail} TOTAL={args.repeat}")
        if fail == 0:
            print("Loopback UART correcto.")
            return 0

        print("Loopback UART con fallas. Revisar puente RX<->TX, baudrate y puerto.")
        return 1
    finally:
        os.close(fd)


if __name__ == "__main__":
    raise SystemExit(main())
