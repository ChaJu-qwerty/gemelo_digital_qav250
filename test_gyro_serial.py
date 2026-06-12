#!/usr/bin/env python3
"""
Visualizador comparativo: Gyro Stand vs Pixhawk IMU (log).

Uso rápido (solo Gyro, sin log):
    python3 test_gyro_serial.py /dev/ttyACM0

Con comparación contra log del Pixhawk:
    python3 test_gyro_serial.py /dev/ttyACM0 /home/bris/ros2_ws/vuelo_log_20260610_213448.txt

Controles:
    Ctrl+C  → salir y mostrar resumen
"""

import sys
import re
import time
import math
import os

# ── Colores ANSI ──────────────────────────────────────────────────────────────
GRN = "\033[92m"
YLW = "\033[93m"
RED = "\033[91m"
CYN = "\033[96m"
BLD = "\033[1m"
RST = "\033[0m"
GRY = "\033[90m"

def detectar_puertos():
    import glob
    return sorted(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*'))

def barra(valor, rango=30, ancho=20):
    """Barra horizontal centrada en 0 para visualizar ángulos."""
    frac = max(-1.0, min(1.0, valor / rango))
    centro = ancho // 2
    pos = int(centro + frac * centro)
    pos = max(0, min(ancho - 1, pos))  # clamp para evitar IndexError
    barra = [' '] * ancho
    barra[centro] = '|'
    if pos != centro:
        lo, hi = min(pos, centro), max(pos, centro)
        for i in range(lo, hi + 1):
            barra[i] = '█'
    color = GRN if abs(valor) < 5 else (YLW if abs(valor) < 15 else RED)
    return color + ''.join(barra) + RST

def cargar_log_pixhawk(path):
    """Lee el log del Pixhawk y devuelve lista de (t, roll, pitch, yaw)."""
    datos = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#') or line.startswith('-') or 't' in line[:5]:
                continue
            parts = line.replace('|', ' ').split()
            if len(parts) < 7:
                continue
            try:
                t, x, y, z = float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3])
                roll, pitch, yaw = float(parts[4]), float(parts[5]), float(parts[6])
                datos.append((t, roll, pitch, yaw))
            except:
                continue
    return datos

def main():
    import serial

    # ── Argumentos ────────────────────────────────────────────────────────────
    puertos = detectar_puertos()
    puerto = sys.argv[1] if len(sys.argv) >= 2 else (puertos[0] if puertos else None)
    log_path = sys.argv[2] if len(sys.argv) >= 3 else None
    baudrate = 9600

    if not puerto:
        print(f"{RED}[ERROR] No se encontró ningún puerto serial.{RST}")
        sys.exit(1)

    # ── Cargar log del Pixhawk si se proporcionó ──────────────────────────────
    log_pixhawk = []
    if log_path and os.path.exists(log_path):
        log_pixhawk = cargar_log_pixhawk(log_path)
        print(f"{GRN}[OK]{RST} Log Pixhawk cargado: {len(log_pixhawk)} muestras de {os.path.basename(log_path)}")
        # Promedios en reposo (primeras muestras con Z=0 o las 50 primeras)
        muestra_reposo = log_pixhawk[:50]
        avg_r = sum(d[1] for d in muestra_reposo) / len(muestra_reposo)
        avg_p = sum(d[2] for d in muestra_reposo) / len(muestra_reposo)
        avg_y = sum(d[3] for d in muestra_reposo) / len(muestra_reposo)
        print(f"    Reposo (promedio 50 primeras muestras): "
              f"Roll={avg_r:+.1f}° Pitch={avg_p:+.1f}° Yaw={avg_y:+.1f}°")
    elif log_path:
        print(f"{YLW}[WARN]{RST} No se encontró el log: {log_path}")

    # ── Patrón regex (igual que nodo_lector_gyro.py) ──────────────────────────
    pattern = re.compile(
        rb"z[\x00-\x1f\x2c]{1,5}"
        rb"([-+]?\d+(?:\.\d+)?)[\x00]*,"
        rb"([-+]?\d+(?:\.\d+)?)[\x00]*,"
        rb"([-+]?\d+(?:\.\d+)?)[\x00]*,\{"
    )

    print()
    print(f"{BLD}{'='*70}{RST}")
    print(f"{BLD}  GYRO STAND LIVE  |  Puerto: {puerto}  |  Ctrl+C para salir{RST}")
    if log_pixhawk:
        print(f"  Comparando con reposo Pixhawk: Roll={avg_r:+.1f}° Pitch={avg_p:+.1f}° Yaw={avg_y:+.1f}°")
    print(f"{BLD}{'='*70}{RST}")

    try:
        ser = serial.Serial(puerto, baudrate, timeout=0.5)
    except Exception as e:
        print(f"{RED}[ERROR] {e}{RST}")
        sys.exit(1)

    buffer = b""
    n = 0
    yaw_offset = None
    t_inicio = time.time()
    ultimos_roll, ultimos_pitch, ultimos_yaw = [], [], []

    try:
        while True:
            chunk = ser.read(512)
            if not chunk:
                continue
            buffer += chunk
            matches = list(pattern.finditer(buffer))
            if not matches:
                if len(buffer) > 1024:
                    buffer = buffer[-512:]
                continue

            match = matches[-1]
            roll_raw  = float(match.group(1))
            pitch_raw = float(match.group(2))
            yaw_raw   = float(match.group(3))

            # Corrección de montaje invertido
            roll_deg  = roll_raw  - 180.0
            pitch_deg = pitch_raw - 180.0

            # Normalizar
            while roll_deg  >  180.0: roll_deg  -= 360.0
            while roll_deg  < -180.0: roll_deg  += 360.0
            while pitch_deg >  180.0: pitch_deg -= 360.0
            while pitch_deg < -180.0: pitch_deg += 360.0
            while yaw_raw   >  180.0: yaw_raw   -= 360.0
            while yaw_raw   < -180.0: yaw_raw   += 360.0

            # Offset de Yaw (primer valor = 0)
            if yaw_offset is None:
                yaw_offset = yaw_raw
            yaw_deg = yaw_raw - yaw_offset
            while yaw_deg >  180.0: yaw_deg -= 360.0
            while yaw_deg < -180.0: yaw_deg += 360.0

            n += 1
            ultimos_roll.append(roll_deg)
            ultimos_pitch.append(pitch_deg)
            ultimos_yaw.append(yaw_deg)
            if len(ultimos_roll) > 10:
                ultimos_roll.pop(0); ultimos_pitch.pop(0); ultimos_yaw.pop(0)

            avg_r_live = sum(ultimos_roll) / len(ultimos_roll)
            avg_p_live = sum(ultimos_pitch) / len(ultimos_pitch)
            avg_y_live = sum(ultimos_yaw) / len(ultimos_yaw)

            # Limpiar pantalla (reescribir las últimas líneas)
            if n > 1:
                # Subir N líneas
                lineas = 12 if log_pixhawk else 8
                print(f"\033[{lineas}A\033[J", end="")

            print(f"{GRY}Paquete #{n:>5}  |  t={time.time()-t_inicio:>6.1f}s{RST}")
            print(f"")
            print(f"  {BLD}GYRO STAND (corregido):{RST}")
            print(f"  Roll  {roll_deg:>+8.2f}°  {barra(roll_deg, 30, 30)}")
            print(f"  Pitch {pitch_deg:>+8.2f}°  {barra(pitch_deg, 30, 30)}")
            print(f"  Yaw   {yaw_deg:>+8.2f}°  {barra(yaw_deg, 180, 30)}")
            print(f"  {GRY}Promedio 10 muestras: Roll={avg_r_live:+.1f}° Pitch={avg_p_live:+.1f}° Yaw={avg_y_live:+.1f}°{RST}")

            if log_pixhawk:
                print(f"")
                print(f"  {BLD}PIXHAWK IMU (reposo del log):{RST}")
                print(f"  Roll  {avg_r:>+8.2f}°  {barra(avg_r, 30, 30)}")
                print(f"  Pitch {avg_p:>+8.2f}°  {barra(avg_p, 30, 30)}")
                print(f"  Yaw   {avg_y:>+8.2f}°  {barra(avg_y, 180, 30)}")
                dR = roll_deg - avg_r
                dP = pitch_deg - avg_p
                dY = yaw_deg - avg_y
                color_d = GRN if max(abs(dR), abs(dP)) < 3 else (YLW if max(abs(dR), abs(dP)) < 8 else RED)
                print(f"  {color_d}Diferencia vs Gyro: ΔRoll={dR:+.1f}° ΔPitch={dP:+.1f}° ΔYaw={dY:+.1f}°{RST}")

            # Limpiar buffer
            ultimo = buffer.rfind(b",{")
            if ultimo != -1:
                buffer = buffer[ultimo + 2:]
            if len(buffer) > 1024:
                buffer = buffer[-512:]

    except KeyboardInterrupt:
        elapsed = time.time() - t_inicio
        print()
        print(f"{BLD}{'='*70}{RST}")
        print(f"  Sesión: {elapsed:.1f}s | Paquetes: {n} | Frecuencia: {n/elapsed:.1f} Hz")
        if ultimos_roll:
            print(f"  Último Gyro (corregido): Roll={ultimos_roll[-1]:+.1f}° "
                  f"Pitch={ultimos_pitch[-1]:+.1f}° Yaw={ultimos_yaw[-1]:+.1f}°")
        print(f"{BLD}{'='*70}{RST}")
        ser.close()

if __name__ == "__main__":
    main()
