# Gemelo Digital QAV250 🚁

Repositorio del **gemelo digital** del cuadricóptero QAV250 montado sobre un banco de pruebas FFT GYRO. El proyecto implementa el modelo matemático de Euler-Lagrange (Luukkonen, 2011) en un nodo de ROS 2, visualizable en tiempo real dentro de Gazebo Classic.

---

## Descripción General

El gemelo digital recibe las señales PWM de los motores directamente desde el Pixhawk mediante MAVLink, integra las ecuaciones de movimiento con RK4 y publica el estado completo del dron (`x, y, z, roll, pitch, yaw`) en ROS 2. Gazebo recibe esa pose y mueve el modelo 3D del dron cinemáticamente.

```
Pixhawk (física)
      │  SERVO_OUTPUT_RAW (MAVLink)
      ▼
captura_pwm.py  ──► PWM → ω (rad/s)
      │
      ▼
modelo_euler_lagrange.py  ──► RK4 Integration
      │                        Estado: x, y, z, φ, θ, ψ
      ▼
nodo_gemelo_digital.py  ──►  /qav250/pose  (PoseStamped)
      │                  ──►  /qav250/motores  (Float32MultiArray)
      │                  ──►  /gazebo/set_entity_state
      ▼
Gazebo Classic 11  ──►  Visualización 3D en tiempo real
```

### Validación con FFT GYRO
El banco FFT GYRO mide los ángulos reales de Roll, Pitch y Yaw via puerto COM. Esos valores se comparan con las estimaciones del modelo para **validar la dinámica rotacional**.

---

## Estado del Proyecto

| Módulo | Estado | Notas |
|--------|--------|-------|
| Modelo Euler-Lagrange (rotacional) | ✅ Completo | Ecuaciones de Luukkonen, X-frame |
| Modelo traslacional (X, Y, Z) | ✅ Completo | Sin validación experimental aún |
| Captura PWM via MAVLink | ✅ Completo | UDP (QGC) y Serial |
| Nodo ROS 2 | ✅ Completo | Multi-hilo, timer adaptativo |
| Modelo Gazebo 3D | ✅ Completo | Cinemático, brazos color-coded |
| Launch file | ✅ Completo | Gazebo + spawn + nodo integrado |
| Parámetros físicos reales | ⏳ Pendiente | k, b: banco de pruebas; Ixx/Iyy/Izz: SolidWorks |
| Datos FFT GYRO via COM port | ⏳ Pendiente | Comparación Roll/Pitch/Yaw |
| Caracterización motor (k, b) | ⏳ En progreso | Pruebas en banco |

---

## Estructura del Repositorio

```
gemelo_digital_qav250/
├── src_tw/                         # Paquete ROS 2 principal
│   ├── gemelo_digital_qav250/      # Código Python
│   │   ├── __init__.py
│   │   ├── captura_pwm.py          # Lectura PWM desde Pixhawk vía MAVLink
│   │   ├── modelo_euler_lagrange.py# Modelo matemático + integración RK4
│   │   ├── nodo_gemelo_digital.py  # Nodo ROS 2 (orquestador principal)
│   │   └── config/
│   │       └── qav250_params.yaml  # Parámetros físicos del dron
│   ├── launch/
│   │   └── gemelo_digital.launch.py# Launch: Gazebo + spawn + nodo
│   ├── models/drone_demo/          # Modelo 3D del dron para Gazebo
│   │   ├── model.config
│   │   └── drone_simple.sdf
│   ├── worlds/
│   │   └── qav250_twin.world       # Mundo de Gazebo con plugin ROS 2
│   ├── test/
│   │   └── test_modelo.py          # Pruebas unitarias del modelo
│   ├── package.xml
│   ├── setup.py
│   ├── setup.cfg
│   └── instalar_dependencias.sh
└── contexto_reto_ia.md             # Descripción del reto (asesor)
```

---

## Requisitos del Sistema

- **OS**: Ubuntu 22.04 LTS
- **ROS**: ROS 2 Humble Hawksbill
- **Simulador**: Gazebo Classic 11 (`gazebo_ros_pkgs`)
- **Python**: 3.10+

---

## Ideas para Completar el Proyecto

### 🔴 Prioritario (bloquea la validación)
1. **Caracterizar motores**: correr pruebas en banco de empuje para obtener la constante `k` real (`F = k·ω²`) y la constante de par `b` (`Q = b·ω²`).
2. **Modelo de inercia en SolidWorks**: obtener `Ixx`, `Iyy`, `Izz` del dron + accesorios montados. Sumar la inercia de la parte móvil del FFT GYRO.
3. **Lectura del FFT GYRO por COM**: agregar un nodo o script que lea Roll/Pitch/Yaw del FFT GYRO y los publique como topic ROS 2 para compararlos con `/qav250/pose`.

### 🟡 Mejoras al Modelo
4. **Offset de empuje en hover**: en el banco FFT GYRO el dron no vuela, pero los motores generan par. Validar que la condición de equilibrio en ω (hover en libre) no genere torques netos en el banco.
5. **Coeficientes de arrastre aerodinámico** (`Ax`, `Ay`, `Az`): aunque el dron esté en el banco, el arrastre del aire afecta los torques. Una primera aproximación es `Az ≈ 0` y los laterales también, pero conviene medirlos en el banco.
6. **Singularidad de gimbal lock**: la formulación Euler-Lagrange con ángulos de Euler se vuelve singular cerca de `θ = ±90°`. Para el banco FFT GYRO esto rara vez ocurre, pero considerar cambiar a cuaterniones si se necesitan maniobras grandes.

### 🟢 Visualización y Análisis
7. **Subscriber de comparación**: agregar un nodo `comparador_angulos.py` que subscriba a `/qav250/pose` (estimado) y al topic de ángulos del FFT GYRO (medido) y calcule el error RMSE en tiempo real.
8. **Grabación de datos**: usar `ros2 bag record` para guardar sesiones completas y analizarlas después en Python/MATLAB.
9. **Panel de monitoreo (RViz)**: crear un archivo de configuración `.rviz` que muestre la trayectoria X,Y,Z estimada, los ángulos y las velocidades de motores simultáneamente.

---

## Guía de Inicio Rápido

Ver el [`src_tw/README.md`](src_tw/README.md) para instrucciones detalladas de instalación, compilación y ejecución.

---

## Referencias

- Luukkonen, T. (2011). *Modelling and control of quadcopter*. Aalto University.
- PX4 Autopilot Motor Conventions: https://docs.px4.io/main/en/airframes/airframe_reference.html
- ROS 2 Humble: https://docs.ros.org/en/humble/
- Gazebo Classic: https://classic.gazebosim.org/
