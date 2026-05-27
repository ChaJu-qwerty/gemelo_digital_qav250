# gemelo_digital_qav250 — Paquete ROS 2

Paquete ROS 2 Humble que implementa el **gemelo digital** del cuadricóptero QAV250. Recibe señales PWM del Pixhawk vía MAVLink, integra las ecuaciones de Euler-Lagrange con RK4 y mueve un modelo 3D en Gazebo Classic 11 en tiempo real.

---

## Estructura del Paquete

```
src_tw/
├── gemelo_digital_qav250/          # Módulo Python principal
│   ├── __init__.py
│   ├── captura_pwm.py              # Conexión MAVLink y conversión PWM→ω
│   ├── modelo_euler_lagrange.py    # Física del dron (Luukkonen 2011)
│   ├── nodo_gemelo_digital.py      # Nodo ROS 2 orquestador
│   └── config/
│       └── qav250_params.yaml      # Parámetros físicos y de conexión
├── launch/
│   └── gemelo_digital.launch.py   # Lanza Gazebo + dron + nodo
├── models/drone_demo/              # Modelo 3D Gazebo
│   ├── model.config
│   └── drone_simple.sdf            # SDF cinemático con brazos color-coded
├── worlds/
│   └── qav250_twin.world           # Mundo Gazebo con plugin gazebo_ros_state
├── test/
│   └── test_modelo.py              # Pruebas unitarias (sin ROS ni Gazebo)
├── package.xml
├── setup.py
├── setup.cfg
├── instalar_dependencias.sh
└── README.md                       # Este archivo
```

---

## Dependencias

### Sistema (apt)
```bash
sudo apt-get install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-msgs \
    ros-humble-tf-transformations \
    ros-humble-launch-ros \
    python3-pip python3-numpy python3-transforms3d
```

### Python (pip)
```bash
pip3 install pymavlink
```

> 💡 También puedes usar el script incluido:
> ```bash
> bash instalar_dependencias.sh
> ```

---

## Instalación y Compilación

### 1. Clonar / Copiar en el workspace de ROS 2

```bash
# Si no tienes workspace, créalo:
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Copia o enlaza este paquete aquí:
# La carpeta src_tw debe quedar como:
#   ~/ros2_ws/src/gemelo_digital_qav250/
cp -r /ruta/al/repo/gemelo_digital_qav250/src_tw ~/ros2_ws/src/gemelo_digital_qav250
```

### 2. Compilar con Colcon

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select gemelo_digital_qav250
```

### 3. Sourcear el workspace

```bash
source ~/ros2_ws/install/setup.bash
```

> ⚠️ Agrega esta línea a tu `~/.bashrc` para no tener que repetirla:
> ```bash
> echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
> ```

---

## Ejecución

### Opción A — Launch completo (Gazebo + Nodo) — Recomendado

```bash
ros2 launch gemelo_digital_qav250 gemelo_digital.launch.py
```

Esto:
1. Lanza Gazebo Classic con `qav250_twin.world`
2. Spawnea el modelo 3D del dron en el origen
3. Inicia el nodo `nodo_gemelo_digital` con los parámetros de `qav250_params.yaml`
4. El nodo espera conexión MAVLink desde el Pixhawk (UDP puerto 14551 por defecto)

### Opción B — Solo el nodo (sin Gazebo)

```bash
ros2 run gemelo_digital_qav250 nodo_gemelo_digital \
  --ros-args --params-file src_tw/gemelo_digital_qav250/config/qav250_params.yaml
```

### Opción C — Solo captura PWM (diagnóstico)

Para verificar que el Pixhawk está enviando datos correctamente:
```bash
# Via UDP (con QGroundControl activo):
ros2 run gemelo_digital_qav250 captura_pwm -- --modo udp

# Via USB directo:
ros2 run gemelo_digital_qav250 captura_pwm -- --modo serial --puerto /dev/ttyACM0

# Guardar a CSV:
ros2 run gemelo_digital_qav250 captura_pwm -- --modo udp --guardar datos_prueba.csv
```

---

## Configuración MAVLink (QGroundControl → UDP)

Para que el nodo reciba datos del Pixhawk via QGC:

1. Abre QGroundControl
2. Ve a **Application Settings → Comm Links → Add**
3. Tipo: **UDP**, Puerto escucha: `14550`
4. Agrega target: `127.0.0.1:14551`
5. El nodo escucha en `udpin:127.0.0.1:14551`

Para conexión directa por USB (sin QGC), edita `qav250_params.yaml`:
```yaml
mavlink_modo: "serial"
mavlink_puerto: "/dev/ttyACM0"   # o /dev/ttyUSB0
```

---

## Topics ROS 2 Publicados

| Topic | Tipo | Contenido |
|-------|------|-----------|
| `/qav250/pose` | `geometry_msgs/PoseStamped` | Posición (x,y,z) + orientación (quaternion) estimada |
| `/qav250/motores` | `std_msgs/Float32MultiArray` | `[ω1, ω2, ω3, ω4, pwm1, pwm2, pwm3, pwm4]` |
| `/gazebo/set_entity_state` | `gazebo_msgs/EntityState` | Pose enviada a Gazebo para mover el modelo |

### Monitorear topics en tiempo real

```bash
# Ver pose del gemelo en la terminal:
ros2 topic echo /qav250/pose

# Ver velocidades de motores:
ros2 topic echo /qav250/motores

# Ver frecuencia real de publicación:
ros2 topic hz /qav250/pose
```

---

## Pruebas Unitarias del Modelo (sin ROS)

Puedes verificar el modelo matemático en **cualquier máquina con Python y NumPy**, sin necesitar ROS ni Gazebo ni el dron físico:

```bash
# Desde la carpeta src_tw:
cd ~/ros2_ws/src/gemelo_digital_qav250
python3 test/test_modelo.py
```

Pruebas incluidas:
- ✅ Hover equilibrado (empuje = peso, torques = 0)
- ✅ Comando de Roll → torque de roll positivo
- ✅ Comando de Pitch → torque de pitch positivo
- ✅ Comando de Yaw → torque de yaw positivo
- ✅ Integración RK4 de 1 segundo → estado físicamente coherente

---

## Modelo Matemático — Resumen

Basado en **Luukkonen (2011)**, configuración en **X** (frame PX4):

### Asignación de motores

```
        FRENTE (+X)
           M1 (CCW)    M3 (CW)
              \        /
               \  ┼  /
               /  ┼  \
              /        \
           M2 (CCW)    M4 (CW)
        ATRÁS (-X)
```

| Motor | Posición | Giro |
|-------|----------|------|
| M1 (Canal 1) | Frontal Derecho | CCW |
| M2 (Canal 2) | Trasero Izquierdo | CCW |
| M3 (Canal 3) | Frontal Izquierdo | CW |
| M4 (Canal 4) | Trasero Derecho | CW |

### Ecuaciones de Fuerzas y Torques

$$T = k(\omega_1^2 + \omega_2^2 + \omega_3^2 + \omega_4^2)$$

$$\tau_\phi = \frac{l}{\sqrt{2}} k (-\omega_1^2 + \omega_2^2 + \omega_3^2 - \omega_4^2)$$

$$\tau_\theta = \frac{l}{\sqrt{2}} k (-\omega_1^2 + \omega_2^2 - \omega_3^2 + \omega_4^2)$$

$$\tau_\psi = b (\omega_1^2 + \omega_2^2 - \omega_3^2 - \omega_4^2)$$

### Vector de Estado (12 variables)

```
estado[0:3]  = x, y, z          (posición inercial [m])
estado[3:6]  = φ, θ, ψ          (roll, pitch, yaw [rad])
estado[6:9]  = ẋ, ẏ, ż          (velocidades [m/s])
estado[9:12] = φ̇, θ̇, ψ̇         (velocidades angulares [rad/s])
```

---

## Parámetros a Calibrar

Edita `gemelo_digital_qav250/config/qav250_params.yaml` con los valores reales:

| Parámetro | Descripción | Fuente |
|-----------|-------------|--------|
| `m` | Masa total [kg] | Báscula |
| `l` | Longitud del brazo [m] | SolidWorks / cinta |
| `Ixx`, `Iyy`, `Izz` | Momentos de inercia [kg·m²] | SolidWorks + inercia FFT GYRO |
| `k` | Coeficiente de empuje [N·s²/rad²] | Banco de pruebas |
| `b` | Coeficiente de arrastre [N·m·s²/rad²] | Banco de pruebas |
| `Ir` | Inercia del rotor [kg·m²] | Datasheet motor RS2205 |

> ⚠️ **Nota crítica sobre el FFT GYRO**: Los momentos de inercia que usa el modelo deben incluir la contribución de la parte móvil del banco de pruebas:
> ```
> I_total = I_dron (SolidWorks) + I_soporte_movil (FFT GYRO)
> ```

---

## Referencias

- Luukkonen, T. (2011). *Modelling and control of quadcopter*. School of Science, Aalto University.
- ROS 2 Humble: https://docs.ros.org/en/humble/
- Gazebo Classic 11: https://classic.gazebosim.org/
- PX4 Motor Conventions: https://docs.px4.io/main/en/airframes/airframe_reference.html
