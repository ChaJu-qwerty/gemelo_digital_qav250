# Gemelo Digital QAV250 🚁

Repositorio del **gemelo digital** del cuadricóptero QAV250 montado sobre el banco de pruebas rotatorio **FFT GYRO**. El proyecto implementa el modelo matemático de Euler-Lagrange (Luukkonen, 2011) en un nodo de ROS 2 y sincroniza el estado en tiempo real dentro de Gazebo Classic 11.

---

## 🛠️ Descripción General

El gemelo digital recibe las señales PWM de los motores directamente desde el Pixhawk del dron físico mediante MAVLink, mapea las señales a velocidades angulares $\omega$ (rad/s), integra las ecuaciones de movimiento usando un integrador Runge-Kutta 4 (RK4) de 20 Hz, y publica la pose estimada (`/qav250/pose`) y el estado de los motores. Gazebo recibe la pose y mueve el modelo 3D del dron de manera asíncrona y no bloqueante.

```text
Pixhawk (Dron Físico)
      │  SERVO_OUTPUT_RAW (MAVLink)
      ▼
captura_pwm.py  ──► PWM -> ω (rad/s)
      │
      ▼
modelo_euler_lagrange.py  ──► Integración RK4
      │                        Estado: x, y, z, roll, pitch, yaw
      ▼
nodo_gemelo_digital.py  ──►  /qav250/pose  (PoseStamped)
      │                  ──►  /qav250/motores  (Float32MultiArray)
      │                  ──►  /gazebo/set_entity_state (Servicio asíncrono)
      ▼
Gazebo Classic 11  ──►  Visualización 3D en tiempo real (brazos color-coded)
```

---

## 📊 Estado del Proyecto

| Módulo | Estado | Notas |
|--------|--------|-------|
| **Modelo Euler-Lagrange** | ✅ Completo | Ecuaciones de Luukkonen en X-frame. |
| **Integrador Numérico RK4** | ✅ Completo | Resolviendo dinámicas traslacionales y rotacionales a 20 Hz. |
| **Captura MAVLink** | ✅ Completo | Soporta comunicación por UDP (con QGC) y Serial directa. |
| **Calibración de Motor ($k, b$)** | ✅ Calibrado | Coeficiente $k = 1.72 \times 10^{-6}$ calibrado para hover al 30% (PWM = 1300 µs) y despegue al 35%, idéntico al dron real. |
| **Sincronización de Gazebo** | ✅ Optimizado | Llamadas asíncronas no bloqueantes. Limpieza de procesos zombies previa al lanzamiento. |
| **Telemetría Humana (`ver_pose`)**| ✅ Completo | Script interactivo que traduce cuaterniones a grados (Roll, Pitch, Yaw) y metros en tiempo real. |
| **Instalador Unificado** | ✅ Completo | `instalar_dependencias.sh` descarga e integra ROS 2 Humble, Gazebo 11, librerías Python y configura variables de entorno. |
| **Inercia del FFT GYRO** | ⏳ Pendiente | Integrar la inercia medida de la parte móvil del soporte al modelo matemático en `qav250_params.yaml`. |
| **Lectura FFT GYRO vía COM** | ⏳ Pendiente | Nodo para suscribir y comparar los ángulos reales medidos por el banco contra los del gemelo. |

---

## 📁 Estructura del Repositorio

```text
gemelo_digital_qav250/
├── src_tw/                         # Carpeta del paquete ROS 2
│   ├── gemelo_digital_qav250/      # Código Python
│   │   ├── __init__.py
│   │   ├── captura_pwm.py          # Captura y mapeo PWM -> ω (rad/s)
│   │   ├── modelo_euler_lagrange.py# Ecuaciones físicas del dron
│   │   ├── nodo_gemelo_digital.py  # Orquestador del nodo ROS 2
│   │   ├── ver_pose.py             # Nodo interactivo de telemetría (m y °)
│   │   └── config/
│   │       └── qav250_params.yaml  # YAML de parámetros físicos y de conexión
│   ├── launch/
│   │   ├── gemelo_digital.launch.py# Launch para modo REAL (MAVLink activado)
│   │   └── gemelo_demo.launch.py      # Launch para modo DEMO (Rutina automática)
│   ├── models/drone_demo/          # Archivos SDF y configuración del modelo 3D
│   │   ├── model.config
│   │   └── drone_simple.sdf
│   ├── worlds/
│   │   └── qav250_twin.world       # Mundo Gazebo
│   ├── test/
│   │   └── test_modelo.py          # Pruebas unitarias de física aislada
│   ├── package.xml
│   ├── setup.py
│   ├── setup.cfg
│   └── instalar_dependencias.sh    # Instala ROS, Gazebo, Python y variables
└── contexto_reto_ia.md             # Documento de contexto del reto
```

---

## ⚙️ Guía de Inicio Rápido

Consulta el archivo [`src_tw/README.md`](src_tw/README.md) para ver la guía detallada paso a paso de instalación, compilación, ejecución y calibración de parámetros físicos del gemelo digital.

---

## 📚 Referencias

- Luukkonen, T. (2011). *Modelling and control of quadcopter*. Aalto University.
- ROS 2 Humble Hawksbill Documentation: https://docs.ros.org/en/humble/
- Gazebo Classic 11 Documentation: https://classic.gazebosim.org/
