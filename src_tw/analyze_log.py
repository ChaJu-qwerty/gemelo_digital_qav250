import pandas as pd

log_file = "/home/bris/ros2_ws/vuelo_log_20260603_194246.txt"

data = []
with open(log_file, "r") as f:
    for line in f:
        if line.startswith("#") or line.startswith("-") or "t |" in line:
            continue
        parts = line.replace("|", "").split()
        if len(parts) >= 15:
            data.append([float(x) for x in parts])

df = pd.DataFrame(data, columns=["t", "X", "Y", "Z", "Roll", "Pitch", "Yaw", "w1", "w2", "w3", "w4", "pwm1", "pwm2", "pwm3", "pwm4"])

# Find the first time Z > 0.1
takeoff = df[df["Z"] > 0.1]
if len(takeoff) > 0:
    print("Takeoff at:")
    print(takeoff.iloc[0])
    
print("\nMax values:")
print("Max Z:", df["Z"].max())
print("Max w1:", df["w1"].max())
print("Max pwm1:", df["pwm1"].max())

# Find when pwm1 != 1000 and pwm1 != 1100
active_pwm = df[(df["pwm1"] != 1000) & (df["pwm1"] != 1100)]
if len(active_pwm) > 0:
    print("\nFirst active PWM:")
    print(active_pwm.iloc[0])
else:
    print("\nPWM never exceeded 1100.")
