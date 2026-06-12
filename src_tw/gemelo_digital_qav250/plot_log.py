import sys
import matplotlib.pyplot as plt

log_file = sys.argv[1]
t, x, y, z = [], [], [], []

with open(log_file, 'r') as f:
    for line in f:
        if line.startswith('#') or '---' in line or ' t |' in line:
            continue
        parts = line.split('|')
        if len(parts) >= 2:
            try:
                time = float(parts[0].strip())
                pos_parts = parts[1].split()
                if len(pos_parts) >= 3:
                    t.append(time)
                    x.append(float(pos_parts[0]))
                    y.append(float(pos_parts[1]))
                    z.append(float(pos_parts[2]))
            except:
                pass

plt.figure(figsize=(12, 4))
plt.subplot(1, 3, 1)
plt.plot(x, y)
plt.title('Trayectoria X-Y')
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.grid()

plt.subplot(1, 3, 2)
plt.plot(t, z)
plt.title('Altura Z vs t')
plt.xlabel('t [s]')
plt.ylabel('Z [m]')
plt.grid()

plt.subplot(1, 3, 3)
plt.plot(t, x, label='X')
plt.plot(t, y, label='Y')
plt.title('X, Y vs t')
plt.xlabel('t [s]')
plt.legend()
plt.grid()

plt.tight_layout()
plt.savefig('/home/bris/.gemini/antigravity/brain/0b360a2f-2e81-4d00-8f00-2ef0870024d0/artifacts/plot.png')
print("Plot saved.")
