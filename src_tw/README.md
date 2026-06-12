# gemelo_digital_qav250 - Paquete ROS 2

Este paquete de ROS 2 Humble implementa el gemelo digital del cuadricoptero QAV250 montado sobre el banco de pruebas rotacional FFT GYRO. Integra las ecuaciones fisicas de Euler-Lagrange mediante metodos de Runge-Kutta 4 (RK4) y sincroniza de manera visual el modelo 3D en Gazebo Classic 11 en tiempo real.

---

## Modos de Operacion

1. **Modo Gemelo Comparativo**: 
   Se conectan dos modulos ROS simultaneos: un dron modelado matematicamente a partir de señales de control PWM via MAVLink, y un dron simulado cinematicamente a partir del hardware Gyro del stand. Ambos convergen en el simulador para estudios de validacion de movimiento.

2. **Modo Rutina Automatica**: 
   Utilizado para probar las cinematicas del modelo sin conexion fisica. Genera perfiles escalonados de actuacion PWM preestablecidos (Despegue, Hover, Roll, Pitch, Yaw, Descenso).

---

## Estructura del Proyecto

```text
src_tw/
├── gemelo_digital_qav250/          # Modulo Python principal del nodo ROS 2
│   ├── __init__.py
│   ├── captura_pwm.py              # Conexion MAVLink y conversion PWM -> omega
│   ├── modelo_euler_lagrange.py    # Resolvedor de fisica de Luukkonen 2011
│   ├── nodo_gemelo_digital.py      # Orquestador del dron matematico MAVLink
│   ├── nodo_lector_gyro.py         # Interface serial para el sensor Gyro MPU
│   ├── nodo_gemelo_fantasma.py     # Orquestador del dron cinematico del Gyro
│   ├── registrar_datos.py          # Script de recoleccion y comparacion de datos
│   └── config/
│       └── qav250_params.yaml      # Parametros fisicos y constantes
├── launch/
│   ├── gemelo_comparativo.launch.py# Inicializacion paralela completa
│   ├── gemelo_digital.launch.py    # Modo Dron Matematico aislado
│   └── gemelo_demo.launch.py       # Modo Rutina Simulada sin Hardware
├── models/drone_demo/              # Modelos 3D e instancias SDF para Gazebo
│   ├── model.config
│   ├── drone_simple.sdf            # Dron primario (Matematico)
│   └── drone_simple_ghost.sdf      # Dron secundario (Cinematico)
├── worlds/
│   └── qav250_twin.world           # Entorno virtual y parametros ambientales
├── test/
│   └── test_modelo.py              # Test unitario de convergencia matematica
├── package.xml
├── setup.py
├── setup.cfg
└── instalar_dependencias.sh
```

---

## Instalacion y Configuracion

### 1. Preparar dependencias

Asegurarse de tener el entorno de ROS 2 Humble preparado. Existe un script automatizado para facilitar la construccion de ambientes limpios:

```bash
cd /home/bris/Desktop/reto/gemelo_digital_qav250/src_tw
bash instalar_dependencias.sh
```

### 2. Estructura y Compilacion Colcon

Integrar el repositorio al entorno de trabajo (workspace) antes de construir:

```bash
mkdir -p ~/ros2_ws/src
cp -r /home/bris/Desktop/reto/gemelo_digital_qav250/src_tw ~/ros2_ws/src/gemelo_digital_qav250
cd ~/ros2_ws

source /opt/ros/humble/setup.bash
colcon build --packages-select gemelo_digital_qav250 --symlink-install
source install/setup.bash
```

---

## Ejecucion Principal

El metodo principal para validacion con el stand fisico (FFT GYRO) es el lazo comparativo:

```bash
ros2 launch gemelo_digital_qav250 gemelo_comparativo.launch.py
```
Esta secuencia levantara el servidor de Gazebo en el entorno especificado, despachara la integracion numerica basada en la telemetria MAVLink, y acoplara el bus serial del sensor para representar a ambos agentes fisicos simultaneamente.

---

## Extraccion de Resultados (Registro)

Para documentar y someter a evaluacion la diferencia entre estimadores, utilizar la funcion de registro tabular:

```bash
ros2 run gemelo_digital_qav250 registrar_datos
```
Esta orden capturara un muestreo temporal constante, escribiendo el formato tabular `.txt` con los valores de translacion y grados eulerianos.

---

## Calibracion y Modificacion Dinamica

Las variables maestras se exponen en `qav250_params.yaml`:

* **`m`** (0.580 kg): Masa combinada ensamble-banco.
* **`Ixx`, `Iyy`, `Izz`**: Tensorial inercial completo.
* **`k`** (1.72e-06): Coeficiente constante de empuje ascendente.
* **`b`** (2.75e-08): Coeficiente resistivo parasito del rotor.

**Nota sobre Calibracion y Offset**: Al evaluar la telemetria resultante, existe un factor de offset inicial (no ajustado a cero absoluto) debido a que no se logro una estabilizacion perfecta a traves del software QGroundControl en tierra. Dicho fenomeno debe tratarse como un sesgo de medicion en el post-procesamiento. Se recomienda estudiar la diferenciacion matematica sobre las mediciones relativas, donde se mantiene la fidelidad metrica.
