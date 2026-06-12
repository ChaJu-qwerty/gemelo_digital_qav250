# Archivos de Modelos 3D para Gazebo

Esta carpeta incluye las representaciones tridimensionales en formato Simulation Description Format (SDF).

## Descripción de Modelos

- `drone_demo/drone_simple.sdf`: Representación base del QAV250. Compuesto por un chasis base simplificado (cajas) e indicadores visuales cilíndricos para diferenciar cada uno de los cuatro rotores. Permite el chequeo instantáneo de asimetría visual en simulaciones de rotación.
- `drone_demo/drone_simple_ghost.sdf`: Variante geométrica exacta del `drone_simple.sdf`, pero con texturas o identificadores translucidos orientados a representar el dron cinemático. Utilizado en paralelismo con el nodo fantasma para empalmar poses tridimensionales del Gyro UART contra las estimaciones del Pixhawk MAVLink.

## Notas sobre Entidades

Los modelos no poseen configuraciones complejas de control PID dinámico dentro del SDF (`gazebo_ros_control`). Dado que el motor matemático en Python se encarga de calcular explícitamente el vector de traslación y rotación absolutos, el SDF actúa únicamente como contenedor cinemático sin propiedades de simulación rígidas habilitadas por motor físico de Gazebo (ODE/Bullet), con el fin de evitar conflictos inerciales iterativos.
