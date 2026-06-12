# Gemelo Digital QAV250

Repositorio del gemelo digital del cuadricoptero QAV250 montado sobre el banco de pruebas rotatorio FFT GYRO. El proyecto implementa el modelo matematico de Euler-Lagrange (Luukkonen, 2011) en un nodo de ROS 2 y sincroniza el estado en tiempo real dentro de Gazebo Classic 11.

---

## Descripcion General del Proyecto

El gemelo digital se compone de dos flujos de datos principales que operan de manera simultanea para permitir la validacion fisica del modelo matematico:

1. **Flujo MAVLink (Dron Principal)**:
   Recibe las senales PWM de los motores directamente desde el Pixhawk del dron fisico mediante el protocolo MAVLink. Estas senales se mapean a velocidades angulares (rad/s) para integrar las ecuaciones de movimiento usando un integrador de tipo Runge-Kutta 4 (RK4) a 20 Hz. El resultado es publicado en ROS 2 como la pose estimada del dron, la cual se envia a Gazebo para visualizacion.

2. **Flujo Serial (Dron Fantasma / Gyro)**:
   Un segundo dron (el dron fantasma) lee directamente la actitud rotacional medida por el sensor Gyro instalado en la base metalica del stand de pruebas, comunicandose via puerto Serial (UART). Esta lectura actua como la "verdad fisica" del comportamiento del ensamble, y se integra a la visualizacion de Gazebo de manera paralela.

Esta arquitectura permite observar y registrar ambos fenomenos de forma asincrona y no bloqueante.

---

## Tecnologias Utilizadas

- **ROS 2 (Foxy / Humble)**: Middleware de comunicacion para la interconexion de los modulos de captura de datos, solucion matematica y entorno grafico.
- **Gazebo Classic 11**: Entorno de simulacion 3D donde se instancian los modelos (SDF) de los drones para validacion visual.
- **Python 3**: Lenguaje base para los nodos de lectura, integracion numerica (RK4) y orquestacion.
- **Pymavlink**: Biblioteca utilizada para la extraccion asincrona de las senales (SERVO_OUTPUT_RAW y ATTITUDE) desde el controlador de vuelo Pixhawk.
- **Pyserial**: Biblioteca para la adquisicion de datos continuos y manejo de reconexion a nivel de hardware con el microcontrolador del Gyro.
- **Numpy**: Computo matricial para la estabilizacion de las rotaciones espaciales en la representacion matricial del modelo euleriano.

---

## Modulos Principales

| Modulo | Descripcion |
|--------|-------------|
| **modelo_euler_lagrange.py** | Implementa las ecuaciones de Luukkonen en X-frame, integrando la dinamica traslacional y rotacional a traves del algoritmo RK4. |
| **captura_pwm.py** | Modulo de recepcion MAVLink. Soporta conexion UDP (en ruteo con QGroundControl) o Serial directo al Pixhawk. |
| **nodo_gemelo_digital.py** | Nodo principal de ROS 2. Procesa la lectura MAVLink, llama al modelo matematico y envia estados al servicio `/set_entity_state` de Gazebo. |
| **nodo_lector_gyro.py** | Nodo puente que asegura la conexion robusta al sensor serial externo (FFT GYRO), publicando los resultados en radianes. |
| **nodo_gemelo_fantasma.py** | Nodo secundario que simula el movimiento del ensamble completo partiendo unicamente de las lecturas del gyro fisico y los actuadores. |
| **registrar_datos.py** | Script orquestador que captura todos los topicos de ambos drones y exporta un archivo tabulado `.txt` para analisis estadistico. |

---

## Consideraciones de Calibracion

Actualmente, existe un pequeño desfase estatico desde el estado de reposo al comparar los resultados del Dron Principal contra el Dron Fantasma. Dicho comportamiento es esperado y se debe a que no fue factible llevar a cabo una calibracion fisica estricta a nivel de horizonte (0.0 grados) dentro del entorno de QGroundControl para los modulos inerciales, por limitaciones durante la estabilizacion mecanica inicial. Se aconseja tomar las lecturas enfocandose en la respuesta dinamica diferencial.

---

## Referencias

- Luukkonen, T. (2011). *Modelling and control of quadcopter*. Aalto University.
- Documentacion oficial de ROS 2 Humble Hawksbill: https://docs.ros.org/en/humble/
- Documentacion oficial de Gazebo Classic 11: https://classic.gazebosim.org/
