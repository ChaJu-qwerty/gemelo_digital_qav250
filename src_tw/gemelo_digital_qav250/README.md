# Nodos y Lógica Python (gemelo_digital_qav250)

Esta carpeta contiene el núcleo lógico del Gemelo Digital estructurado como un paquete Python compatible con ROS 2.

## Descripción de Módulos

- `captura_pwm.py`: Administra la conexión asíncrona mediante el protocolo MAVLink (sobre UDP o Serial) al controlador Pixhawk. Interpreta la señal `SERVO_OUTPUT_RAW`, filtrando el ruido e interrupciones, y mapea la señal PWM (1000-2000 µs) a velocidad angular (rad/s).
- `modelo_euler_lagrange.py`: Módulo matemático desacoplado de ROS. Implementa la integración numérica por Runge-Kutta de 4to orden (RK4) para la física traslacional y rotacional descrita por Teppo Luukkonen (2011).
- `nodo_gemelo_digital.py`: Orquestador principal. Mantiene el ciclo temporal a 20Hz, coordina las llamadas a `modelo_euler_lagrange.py` enviando los estados de actuadores y transmite la postura resultante a Gazebo mediante la interfaz `gazebo_msgs`.
- `nodo_lector_gyro.py`: Interfaz serial de conexión directa al MPU6050 del stand de pruebas (FFT GYRO). Incluye algoritmos para manejar microdesconexiones, vaciado de buffer serial e inicialización robusta para Linux.
- `nodo_gemelo_fantasma.py`: Implementación secundaria del modelo cinemático. Lee los ángulos provenientes de `nodo_lector_gyro.py` y proyecta en Gazebo un dron paralelo que sigue el comportamiento del hardware físico.
- `registrar_datos.py`: Tarea utilitaria que se suscribe simultáneamente a los tópicos MAVLink y Gyro, almacenando vectores en formato de texto plano (`.txt`) para posterior evaluación del error métrico.
- `ver_pose.py`: Cliente por terminal que imprime telemetría en tiempo real convirtiendo las variables de cuaterniones a grados (Roll, Pitch, Yaw) y metros.

## Ejecución Aislada

La mayoría de los scripts están diseñados como nodos completos de ROS 2 y pueden ser probados individualmente, por ejemplo:
`ros2 run gemelo_digital_qav250 nodo_lector_gyro`
