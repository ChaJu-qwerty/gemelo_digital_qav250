# gemelo_digital_qav250 — Paquete ROS 2

Este paquete de ROS 2 Humble implementa el gemelo digital del cuadricóptero QAV250 montado sobre el banco de pruebas rotacional FFT GYRO. Integra las ecuaciones físicas de Euler-Lagrange mediante métodos de Runge-Kutta 4 (RK4) y sincroniza de manera visual el modelo 3D en Gazebo Classic 11 en tiempo real.

---

## 🚀 Cambios y Mejoras Recientes

1. **Calibración Física del Empuje (k)**: Se ajustó el coeficiente de empuje a k = 1.72 x 10^-6 en qav250_params.yaml. Esto alinea la simulación para que el hover del dron ocurra al 30% de aceleración (PWM = 1300 µs) y empiece a subir al 35% (PWM = 1350 µs), idéntico al comportamiento real del dron. A velocidades de ralentí/armado (PWM ~1084 µs / 8.4% de throttle), el dron ya no despega ni deriva del suelo en la simulación.
2. **Script de Telemetría en Tiempo Real (ver_pose)**: Se implementó una herramienta de consola que lee el topic /qav250/pose, convierte automáticamente el cuaternión de orientación a ángulos de Euler en Grados (Roll, Pitch, Yaw) y mantiene la posición en Metros, refrescando la consola de forma limpia.
3. **Cierre Automático de Procesos Zombies**: Se incorporó un limpiador automático en los archivos de lanzamiento (launch) para matar cualquier instancia previa huérfana de gzserver o gzclient antes de iniciar la simulación, resolviendo los cuelgues de inicialización y errores exit code 255 de Gazebo.
4. **Física No Bloqueante**: La comunicación del nodo con el servicio /set_entity_state de Gazebo se rediseñó a llamadas asíncronas libres de bloqueos. Si Gazebo se ralentiza al iniciar, el solucionador matemático del gemelo digital continúa procesando la física de fondo sin interrupciones a sus 20 Hz nominales.
5. **Script de Instalación Unificado**: instalar_dependencias.sh ahora instala e integra de forma automática todo el entorno necesario (ROS 2 Humble, Gazebo Classic 11, dependencias del sistema y de Python).

---

## 📁 Estructura del Proyecto

```text
src_tw/
├── gemelo_digital_qav250/          # Módulo Python principal del nodo ROS 2
│   ├── __init__.py
│   ├── captura_pwm.py              # Conexión MAVLink y conversión PWM -> ω (rad/s)
│   ├── modelo_euler_lagrange.py    # Resolvedor de física (Ecuaciones de Luukkonen 2011)
│   ├── nodo_gemelo_digital.py      # Nodo ROS 2 orquestador principal
│   ├── ver_pose.py                 # Nodo de telemetría Roll/Pitch/Yaw en grados y metros
│   └── config/
│       └── qav250_params.yaml      # Parámetros físicos, aerodinámicos y de conexión
├── launch/
│   ├── gemelo_digital.launch.py   # Launch para modo REAL (conecta a Pixhawk vía MAVLink)
│   └── gemelo_demo.launch.py      # Launch para modo DEMO (simula rutina de vuelo automática)
├── models/drone_demo/              # Modelo 3D y descripción visual del dron para Gazebo
│   ├── model.config
│   └── drone_simple.sdf            # SDF cinemático con hélices y brazos identificados
├── worlds/
│   └── qav250_twin.world           # Mundo de Gazebo con plugin 'gazebo_ros_state'
├── test/
│   └── test_modelo.py              # Test unitario del modelo Euler-Lagrange (sin dependencias de ROS)
├── package.xml                     # Metadatos del paquete ROS 2
├── setup.py                        # Script de instalación setuptools y scripts ejecutables
├── setup.cfg
└── instalar_dependencias.sh        # Instala ROS, Gazebo, librerías y configura variables
```

---

## 🛠️ Instalación y Configuración Completa

### 1. Instalar dependencias del sistema y de Python
Ejecuta el script unificado de instalación. Este instalará ROS 2 Humble, Gazebo Classic 11 y todos los paquetes requeridos si no los tienes en tu sistema:
```bash
cd /home/bris/Desktop/reto/gemelo_digital_qav250/src_tw
bash instalar_dependencias.sh
```

### 2. Cargar tu terminal e inicializar Workspace de ROS 2
```bash
# Crear o asegurar el workspace de ROS 2
mkdir -p ~/ros2_ws/src
cp -r /home/bris/Desktop/reto/gemelo_digital_qav250/src_tw ~/ros2_ws/src/gemelo_digital_qav250
cd ~/ros2_ws

# Compilar usando colcon
source /opt/ros/humble/setup.bash
colcon build --packages-select gemelo_digital_qav250 --symlink-install
source install/setup.bash
```

---

## 🚀 Ejecución del Proyecto

### Modo A: Vuelo Real (Pixhawk Físico)
Esta opción se conecta al Pixhawk del dron real a través de MAVLink (por puerto serie USB o telemetría UDP).
```bash
ros2 launch gemelo_digital_qav250 gemelo_digital.launch.py
```
*El nodo leerá los PWMs que genera el Pixhawk en tiempo real, calculará la posición teórica en base a la física y la enviará a Gazebo para sincronizar la maqueta.*

### Modo B: Rutina Demostración (Simulado)
Esta opción realiza un vuelo automático prefijado (Despegue, Hover, Roll, Pitch, Yaw, Descenso y Apagado) sin necesidad de conectar hardware real.
```bash
ros2 launch gemelo_digital_qav250 gemelo_demo.launch.py
```

---

## 📊 Telemetría y Monitoreo de Datos

Para verificar la orientación y la posición del dron sin lidiar con los complejos cuaterniones, abre una nueva terminal y corre el nodo de telemetría:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run gemelo_digital_qav250 ver_pose
```

### Unidades utilizadas:
* **Posición (X, Y, Z)**: Expresadas en Metros (m).
* **Orientación (Roll, Pitch, Yaw)**: Expresadas en Grados (°).

```text
==================================================
        TELEMETRÍA DE POSICIÓN Y ORIENTACIÓN      
==================================================
  [UNIDADES DE POSICIÓN: METROS (m)]
    X (Lateral/Roll):      0.005 m
    Y (Longitud/Pitch):   -0.012 m
    Z (Altura/Empuje):     0.320 m
--------------------------------------------------
  [UNIDADES DE ORIENTACIÓN: GRADOS (°)]
    Roll  (Alabeo):        1.20°
    Pitch (Cabeceo):      -0.50°
    Yaw   (Guiñada):      15.30°
==================================================
```

---

## ⚙️ Calibración de Parámetros Físicos

El comportamiento dinámico se puede reconfigurar de manera inmediata editando el archivo qav250_params.yaml (los cambios surten efecto al reiniciar el nodo gracias a --symlink-install):

* **m** (0.580 kg): Masa total medida en báscula.
* **Ixx, Iyy, Izz**: Momentos de inercia del sistema combinado. Nota: Deben sumar los momentos calculados en SolidWorks del dron más la inercia que añade la parte móvil del soporte FFT GYRO en sus respectivos ejes.
* **k** (1.72 x 10^-6): Coeficiente de empuje. Calibrado para hover al 30%.
* **b** (2.75 x 10^-8): Coeficiente de torque de Yaw del motor.
* **Ax, Ay, Az** (0.15): Coeficiente de arrastre traslacional del aire (evita el deslizamiento infinito por inercia).
