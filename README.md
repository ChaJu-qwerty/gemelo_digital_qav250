# Gemelo Digital QAV250 — Validación de Modelado Físico

<div align="center">

![Python](https://img.shields.io/badge/Python-3.10+-3776AB?style=for-the-badge&logo=python&logoColor=white)
![ROS2](https://img.shields.io/badge/ROS2-Humble-22314E?style=for-the-badge&logo=ros&logoColor=white)
![Gazebo](https://img.shields.io/badge/Gazebo-Classic_11-F58025?style=for-the-badge)
![MAVLink](https://img.shields.io/badge/MAVLink-Protocol-5C2D91?style=for-the-badge)
![NumPy](https://img.shields.io/badge/NumPy-Integración_RK4-013243?style=for-the-badge&logo=numpy&logoColor=white)

**· Diseño y validación de dinámica de VANTs ·**

*Simulación de cuadricóptero QAV250 con integración matemática RK4 y telemetría por hardware (Pixhawk + Gyro) en tiempo real.*

</div>

---

## Tabla de Contenidos

- [Descripción General](#descripcion-general)
- [Arquitectura del Sistema](#arquitectura-del-sistema)
- [Tecnologías Utilizadas](#tecnologias-utilizadas)
- [Módulos Principales](#modulos-principales)
- [Consideraciones de Calibración](#consideraciones-de-calibracion)
- [Referencias](#referencias)

---

## Descripción General <a name="descripcion-general"></a>

El gemelo digital se compone de dos flujos de datos principales que operan de manera simultánea para permitir la validación física del modelo matemático implementado (Euler-Lagrange, Luukkonen 2011) sobre el banco de pruebas rotatorio **FFT GYRO**:

1. **Flujo MAVLink (Dron Principal)**:
   Recibe las señales PWM de los motores directamente desde el Pixhawk del dron físico mediante el protocolo MAVLink. Estas señales se mapean a velocidades angulares (rad/s) para integrar las ecuaciones de movimiento usando un integrador de tipo Runge-Kutta 4 (RK4) a 20 Hz. El resultado es publicado en ROS 2 como la pose estimada del dron, la cual se envía a Gazebo para visualización.

2. **Flujo Serial (Dron Fantasma / Gyro)**:
   Un segundo dron (el dron fantasma) lee directamente la actitud rotacional medida por el sensor Gyro instalado en la base metálica del stand de pruebas, comunicándose vía puerto Serial (UART). Esta lectura actúa como la "verdad física" del comportamiento del ensamble, y se integra a la visualización de Gazebo de manera paralela.

Esta arquitectura permite observar y registrar ambos fenómenos de forma asíncrona y no bloqueante.

---

## Arquitectura del Sistema <a name="arquitectura-del-sistema"></a>

```text
┌─────────────────────────────────────────────────────────────┐
│                     ENTRADAS FÍSICAS                        │
│     Dron Principal (Pixhawk)   ·   Dron Fantasma (Gyro)     │
└──────────────────────┬────────────────────────┬─────────────┘
                       │                        │
         ┌─────────────▼─────────────┐  ┌───────▼─────────────┐
         │     captura_pwm.py        │  │ nodo_lector_gyro.py │
         │   MAVLink (UDP/Serial)    │  │ UART Serial (COM)   │
         └─────────────┬─────────────┘  └───────┬─────────────┘
                       │                        │
         ┌─────────────▼─────────────┐          │
         │ modelo_euler_lagrange.py  │          │
         │ Integración RK4 (20 Hz)   │          │
         └─────────────┬─────────────┘          │
                       │                        │
         ┌─────────────▼─────────────┐  ┌───────▼─────────────┐
         │  nodo_gemelo_digital.py   │  │nodo_gemelo_fantasma │
         │  Publica Pose / Motores   │  │ Publica Pose Gyro   │
         └─────────────┬─────────────┘  └───────┬─────────────┘
                       │                        │
┌──────────────────────▼────────────────────────▼─────────────┐
│                     SIMULACIÓN 3D                           │
│        Gazebo Classic 11 (/set_entity_state async)          │
└─────────────────────────────────────────────────────────────┘
```

---

## Tecnologías Utilizadas <a name="tecnologias-utilizadas"></a>

- **ROS 2 (Humble Hawksbill)**: Middleware de comunicación para la interconexión de los módulos de captura de datos, solución matemática y entorno gráfico.
- **Gazebo Classic 11**: Entorno de simulación 3D donde se instancian los modelos (SDF) de los drones para validación visual.
- **Python 3.10**: Lenguaje base para los nodos de lectura, integración numérica (RK4) y orquestación.
- **Pymavlink**: Biblioteca utilizada para la extracción asíncrona de las señales (`SERVO_OUTPUT_RAW` y `ATTITUDE`) desde el controlador de vuelo Pixhawk.
- **Pyserial**: Biblioteca para la adquisición de datos continuos y manejo de reconexión a nivel de hardware con el microcontrolador del Gyro.
- **NumPy**: Cómputo matricial para la estabilización de las rotaciones espaciales en la representación matricial del modelo euleriano.

---

## Módulos Principales <a name="modulos-principales"></a>

| Módulo | Descripción |
|--------|-------------|
| **modelo_euler_lagrange.py** | Implementa las ecuaciones de Luukkonen en X-frame, integrando la dinámica traslacional y rotacional a través del algoritmo RK4. |
| **captura_pwm.py** | Módulo de recepción MAVLink. Soporta conexión UDP (en ruteo con QGroundControl) o Serial directo al Pixhawk. |
| **nodo_gemelo_digital.py** | Nodo principal de ROS 2. Procesa la lectura MAVLink, llama al modelo matemático y envía estados al servicio `/set_entity_state` de Gazebo. |
| **nodo_lector_gyro.py** | Nodo puente que asegura la conexión robusta al sensor serial externo (FFT GYRO), publicando los resultados en radianes. |
| **nodo_gemelo_fantasma.py** | Nodo secundario que simula el movimiento del ensamble completo partiendo únicamente de las lecturas del gyro físico y los actuadores. |
| **registrar_datos.py** | Script orquestador que captura todos los tópicos de ambos drones y exporta un archivo tabulado `.txt` para análisis estadístico. |

---

## Consideraciones de Calibración <a name="consideraciones-de-calibracion"></a>

Actualmente, existe un pequeño desfase estático desde el estado de reposo al comparar los resultados del Dron Principal contra el Dron Fantasma. Dicho comportamiento es esperado y se debe a que no fue factible llevar a cabo una calibración física estricta a nivel de horizonte (0.0 grados) dentro del entorno de QGroundControl para los módulos inerciales, por limitaciones durante la estabilización mecánica inicial. Se aconseja tomar las lecturas enfocándose en la respuesta dinámica diferencial.

---

## Referencias <a name="referencias"></a>

- Luukkonen, T. (2011). *Modelling and control of quadcopter*. Aalto University.
- Documentación oficial de ROS 2 Humble Hawksbill: https://docs.ros.org/en/humble/
- Documentación oficial de Gazebo Classic 11: https://classic.gazebosim.org/
