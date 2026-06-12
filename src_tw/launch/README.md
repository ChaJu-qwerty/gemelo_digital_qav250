# Archivos Launch de ROS 2

Este directorio contiene los scripts de inicialización orquestada del entorno. 

## Archivos Principales

- `gemelo_comparativo.launch.py`: Orquestador definitivo para validación experimental. Inicia Gazebo Classic y lanza paralelamente la simulación matemática y la lectura Gyro.
  **Comando de ejecución:** `ros2 launch gemelo_digital_qav250 gemelo_comparativo.launch.py`
- `gemelo_digital.launch.py`: Lanza exclusivamente el dron matemático leyendo del Pixhawk (MAVLink).
  **Comando de ejecución:** `ros2 launch gemelo_digital_qav250 gemelo_digital.launch.py`
- `gemelo_demo.launch.py`: Simulador que descarta entradas externas e inyecta señales PWM artificiales para validar visualmente la física sin hardware.
  **Comando de ejecución:** `ros2 launch gemelo_digital_qav250 gemelo_demo.launch.py`

## Dependencias Requeridas
Los scripts asumen la correcta instalación del metapaquete `gazebo_ros_pkgs`. Internamente, utilizan comandos POSIX del sistema operativo (por ejemplo, `killall gzserver`) para asegurar que el ambiente se encuentre despejado previo a instanciar la simulación y evitar bloqueos en el puerto 11345.
