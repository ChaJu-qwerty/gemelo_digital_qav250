import pandas as pd
import numpy as np

log_file = "/home/bris/ros2_ws/vuelo_log_20260603_195551.txt"

data = []
with open(log_file, "r") as f:
    for line in f:
        if line.startswith("#") or line.startswith("-") or "t |" in line:
            continue
        parts = line.replace("|", "").split()
        if len(parts) >= 15:
            data.append([float(x) for x in parts])

df = pd.DataFrame(data, columns=["t", "X", "Y", "Z", "Roll", "Pitch", "Yaw", "w1", "w2", "w3", "w4", "pwm1", "pwm2", "pwm3", "pwm4"])

print("--- ANALISIS DE VUELO ---")
print("Tiempo total:", df['t'].max(), "s")
print(f"Desplazamiento final: X={df['X'].iloc[-1]:.2f}, Y={df['Y'].iloc[-1]:.2f}, Z={df['Z'].iloc[-1]:.2f}")
print(f"Maximos de posicion: X={df['X'].max():.2f}, min X={df['X'].min():.2f}")
print(f"                     Y={df['Y'].max():.2f}, min Y={df['Y'].min():.2f}")
print(f"                     Z={df['Z'].max():.2f}, min Z={df['Z'].min():.2f}")
print(f"Maximos de angulos: Roll={df['Roll'].max():.2f}, Pitch={df['Pitch'].max():.2f}, Yaw={df['Yaw'].max():.2f}")

# Promedio de PWMs cuando estubo activo
active = df[df['pwm1'] > 1050]
if len(active) > 0:
    print("\n--- PROMEDIOS EN VUELO ---")
    print(f"Promedio PWM: {active['pwm1'].mean():.0f}, {active['pwm2'].mean():.0f}, {active['pwm3'].mean():.0f}, {active['pwm4'].mean():.0f}")
    print(f"Promedio W: {active['w1'].mean():.0f}, {active['w2'].mean():.0f}, {active['w3'].mean():.0f}, {active['w4'].mean():.0f}")
else:
    print("PWM nunca excedio 1050")

