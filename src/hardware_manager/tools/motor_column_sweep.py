#!/usr/bin/env python3
"""
motor_column_sweep.py
---------------------
Prueba de comunicación UART con el ESP32.

Recorre las 10 columnas de la grilla 5x10 de derecha a izquierda
(col 9 → col 0, igual que el índice de la trama M).
En cada paso enciende al 100 una columna entera y apaga el resto,
luego espera DELAY segundos antes de pasar a la siguiente.

Protocolo: "M d0 d1 ... d49\n"
  índice: (9 - col) * 5 + row   →  col=9,row=0 → d0 (sup-der)
                                    col=0,row=4 → d49 (inf-izq)

Uso:
    python3 motor_column_sweep.py [--port /dev/ttyUSB0] [--baud 115200] [--delay 0.5] [--duty 100]
"""

import argparse
import time
import serial

ROWS = 5
COLS = 10
N    = ROWS * COLS   # 50


def build_frame(duties: list[int]) -> bytes:
    """Construye la trama ASCII 'M d0 d1 ... d49\\n'."""
    assert len(duties) == N
    return ("M " + " ".join(str(d) for d in duties) + "\n").encode("ascii")


def col_frame(active_col: int, duty: int) -> bytes:
    """Trama con una sola columna activa al duty dado, el resto en 0."""
    duties = [0] * N
    for row in range(ROWS):
        idx = (COLS - 1 - active_col) * ROWS + row
        duties[idx] = duty
    return build_frame(duties)


def all_off_frame() -> bytes:
    return build_frame([0] * N)


def main():
    parser = argparse.ArgumentParser(description="Barrido de columnas para ESP32")
    parser.add_argument("--port",  default="/dev/ttyACM0", help="Puerto serie (default: /dev/ttyACM0)")
    parser.add_argument("--baud",  type=int, default=115200, help="Baudrate (default: 115200)")
    parser.add_argument("--delay", type=float, default=0.8, help="Segundos por columna (default: 0.8)")
    parser.add_argument("--duty",  type=int, default=100, help="Duty cycle activo (default: 100)")
    parser.add_argument("--repeat", type=int, default=1, help="Veces que repite el barrido completo (default: 1)")
    args = parser.parse_args()

    print(f"Abriendo {args.port} @ {args.baud} bps")
    with serial.Serial(args.port, args.baud, timeout=1) as ser:
        time.sleep(0.1)  # esperar reset del ESP32 si aplica

        for rep in range(args.repeat):
            print(f"\n── Barrido {rep + 1}/{args.repeat} ──")
            # col 9 (derecha) → col 0 (izquierda), igual que el orden de la trama
            for col in range(COLS - 1, -1, -1):
                frame = col_frame(col, args.duty)
                ser.write(frame)
                print(f"  col {col:2d}  →  {frame.decode().strip()}")
                time.sleep(args.delay)

        # Apagar todo al terminar
        ser.write(all_off_frame())
        print("\nTodos los motores apagados. Listo.")


if __name__ == "__main__":
    main()
