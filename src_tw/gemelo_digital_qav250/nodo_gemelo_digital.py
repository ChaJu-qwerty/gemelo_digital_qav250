import sys
import time
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
# pyrefly: ignore [missing-import]
from geometry_msgs.msg import PoseStamped
# pyrefly: ignore [missing-import]
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, Bool

try:
    from tf_transformations import quaternion_from_euler
except ImportError:
    print("[ERROR] tf_transformations no encontrado.")
    print("        Instalar: sudo apt install ros-humble-tf-transformations")
    sys.exit(1)

try:
    from pymavlink import mavutil
except ImportError:
    # pymavlink no es necesario en modo demo
    mavutil = None

# ── Importar módulos del paquete ──────────────────────────────────────────────
from .modelo_euler_lagrange import DroneModel

try:
    from .captura_pwm_pixhawk import pwm_a_omega, pwm_a_throttle_pct, CANAL_MOTOR
except ImportError:
    from .captura_pwm import pwm_a_omega, pwm_a_throttle_pct, CANAL_MOTOR

try:
    from gazebo_msgs.msg import EntityState
    from gazebo_msgs.srv import SetEntityState
    HAS_GAZEBO_MSGS = True
except ImportError:
    HAS_GAZEBO_MSGS = False


# ── Convención de motores QAV250 frame X (PX4) ───────────────────────────────
#   Canal SERVO_OUTPUT_RAW -> Motor -> Sentido de giro
#   1 -> Frontal derecho   -> CCW  (omega_G = +omega1)
#   2 -> Trasero izquierdo -> CCW  (omega_G = +omega2)
#   3 -> Frontal izquierdo -> CW   (omega_G = -omega3)
#   4 -> Trasero derecho   -> CW   (omega_G = -omega4)
CANALES_MOTOR = [1, 2, 3, 4]


class NodoGemeloDigital(Node):
    """
    Nodo ROS 2 que conecta el Pixhawk físico (o rutina demo) con el modelo
    matemático y publica el estado resultante para Gazebo.
    
    Modos de operación:
    - modo_demo=false (default): Conecta al Pixhawk vía MAVLink
    - modo_demo=true: Lee PWM del topic /qav250/pwm_demo (publicado por rutina_demo.py)
    """

    def __init__(self):
        super().__init__("nodo_gemelo_digital")

        # Declarar y cargar parámetros desde YAML
        self._declarar_parametros()
        params = self._cargar_parametros()

        self.get_logger().info("Parámetros cargados desde YAML:")
        for k, v in params.items():
            self.get_logger().info(f"   {k} = {v}")

        # Inicializar modelo matemático
        parametros_modelo = {
            "m"  : params["m"],
            "l"  : params["l"],
            "Ixx": params["Ixx"],
            "Iyy": params["Iyy"],
            "Izz": params["Izz"],
            "k"  : params["k"],
            "b"  : params["b"],
            "Ir" : params["Ir"],
            "Ax" : params["Ax"],
            "Ay" : params["Ay"],
            "Az" : params["Az"],
        }
        self.modelo = DroneModel(parametros_modelo)
        self.get_logger().info("Modelo Euler-Lagrange inicializado.")

        # ── Publishers ────────────────────────────────────────────────────────
        self.pub_pose = self.create_publisher(
            PoseStamped,
            params["topic_pose"],
            10,
        )
        self.pub_motores = self.create_publisher(
            Float32MultiArray,
            params["topic_motores"],
            10,
        )
        self.get_logger().info(f"Publicando pose en:    {params['topic_pose']}")
        self.get_logger().info(f"Publicando motores en: {params['topic_motores']}")

        # Service client opcional hacia Gazebo
        if HAS_GAZEBO_MSGS:
            self.cli_gazebo = self.create_client(
                SetEntityState,
                "/set_entity_state"
            )
            self.get_logger().info("Gazebo SetEntityState client activo: /set_entity_state")
        else:
            self.cli_gazebo = None
            self.get_logger().warn("gazebo_msgs no disponible — sin comunicación con Gazebo.")

        # ── Estado interno ────────────────────────────────────────────────────
        self.estado     = np.zeros(12)
        self.t_anterior = None
        self.dt_nominal = 1.0 / params["frecuencia_hz"]
        self.params     = params

        # Calcular OMEGA_MAX dinámicamente según los parámetros del motor del YAML
        motor_kv = self.params.get("motor_kv", 2300.0)
        bateria_voltaje = self.params.get("bateria_voltaje", 14.8)
        esc_eficiencia = self.params.get("esc_eficiencia", 0.85)
        rpm_max = motor_kv * bateria_voltaje * esc_eficiencia
        self.omega_max = rpm_max * (2.0 * np.pi / 60.0)

        # PWM actual de cada motor (actualizado desde el hilo MAVLink o demo)
        self.pwm_actual = [1000.0] * 4
        self._lock      = threading.Lock()
        
        # ── Contadores de diagnóstico ─────────────────────────────────────────
        self._nan_reset_count = 0

        # ── Modo Demo o MAVLink ───────────────────────────────────────────────
        self.modo_demo = params.get("modo_demo", False)
        
        if self.modo_demo:
            # ── MODO DEMO: subscribirse al topic de PWM simulado ──────────────
            self.get_logger().info("=" * 60)
            self.get_logger().info("  MODO DEMO ACTIVADO — Sin conexión MAVLink")
            self.get_logger().info("  Esperando PWM en /qav250/pwm_demo")
            self.get_logger().info("  Ejecutar: ros2 run gemelo_digital_qav250 rutina_demo")
            self.get_logger().info("=" * 60)
            
            self.sub_demo = self.create_subscription(
                Float32MultiArray,
                "/qav250/pwm_demo",
                self._callback_pwm_demo,
                10,
            )
            # Reset de estado al inicio de cada ciclo de la rutina demo
            self.sub_reset = self.create_subscription(
                Bool,
                "/qav250/reset_modelo",
                self._callback_reset_modelo,
                10,
            )
            self.conexion_mav = None
        else:
            # ── MODO REAL: Conexión MAVLink en hilo separado ──────────────────
            if mavutil is None:
                self.get_logger().error(
                    "pymavlink no instalado y modo_demo=false. "
                    "Instalar: pip3 install pymavlink, o usar modo_demo=true"
                )
                return
                
            self.conexion_mav = None
            self._hilo_mavlink = threading.Thread(
                target=self._bucle_mavlink,
                daemon=True,
                name="hilo_mavlink",
            )
            self._hilo_mavlink.start()

        # ── Timer principal del nodo ──────────────────────────────────────────
        # Corre a la frecuencia definida en YAML y publica el estado
        self.timer = self.create_timer(
            self.dt_nominal,
            self._callback_publicar,
        )
        self.get_logger().info(
            f"Nodo listo a {params['frecuencia_hz']} Hz. "
            f"{'Modo DEMO' if self.modo_demo else 'Esperando datos MAVLink'}..."
        )

    # ── PARÁMETROS ────────────────────────────────────────────────────────────

    def _declarar_parametros(self):
        """Declarar todos los parámetros que puede recibir del YAML."""
        self.declare_parameter("m",                  0.650)
        self.declare_parameter("l",                  0.125)
        self.declare_parameter("Ixx",                0.0058)
        self.declare_parameter("Iyy",                0.0058)
        self.declare_parameter("Izz",                0.0100)
        self.declare_parameter("k",                  3.13e-5)
        self.declare_parameter("b",                  7.50e-7)
        self.declare_parameter("Ir",                 6.00e-5)
        self.declare_parameter("Ax",                 0.0)
        self.declare_parameter("Ay",                 0.0)
        self.declare_parameter("Az",                 0.0)
        self.declare_parameter("mavlink_modo",       "udp")
        self.declare_parameter("mavlink_puerto",     "/dev/ttyACM0")
        self.declare_parameter("mavlink_udp_ip",     "0.0.0.0")
        self.declare_parameter("mavlink_udp_puerto", 14551)
        self.declare_parameter("frecuencia_hz",      20.0)
        self.declare_parameter("topic_pose",         "/qav250/pose")
        self.declare_parameter("topic_motores",      "/qav250/motores")
        self.declare_parameter("motor_kv",           2300.0)
        self.declare_parameter("bateria_voltaje",    14.8)
        self.declare_parameter("esc_eficiencia",     0.85)
        self.declare_parameter("modo_demo",          False)

    def _cargar_parametros(self) -> dict:
        """Lee todos los parámetros declarados y los retorna como dict."""
        return {
            "m"                 : self.get_parameter("m").value,
            "l"                 : self.get_parameter("l").value,
            "Ixx"               : self.get_parameter("Ixx").value,
            "Iyy"               : self.get_parameter("Iyy").value,
            "Izz"               : self.get_parameter("Izz").value,
            "k"                 : self.get_parameter("k").value,
            "b"                 : self.get_parameter("b").value,
            "Ir"                : self.get_parameter("Ir").value,
            "Ax"                : self.get_parameter("Ax").value,
            "Ay"                : self.get_parameter("Ay").value,
            "Az"                : self.get_parameter("Az").value,
            "mavlink_modo"      : self.get_parameter("mavlink_modo").value,
            "mavlink_puerto"    : self.get_parameter("mavlink_puerto").value,
            "mavlink_udp_ip"    : self.get_parameter("mavlink_udp_ip").value,
            "mavlink_udp_puerto": self.get_parameter("mavlink_udp_puerto").value,
            "frecuencia_hz"     : self.get_parameter("frecuencia_hz").value,
            "topic_pose"        : self.get_parameter("topic_pose").value,
            "topic_motores"     : self.get_parameter("topic_motores").value,
            "motor_kv"          : self.get_parameter("motor_kv").value,
            "bateria_voltaje"   : self.get_parameter("bateria_voltaje").value,
            "esc_eficiencia"    : self.get_parameter("esc_eficiencia").value,
            "modo_demo"         : self.get_parameter("modo_demo").value,
        }

    # ── CALLBACK PWM DEMO ─────────────────────────────────────────────────────

    def _callback_pwm_demo(self, msg: Float32MultiArray):
        """Recibe PWM simulados del nodo rutina_demo."""
        if len(msg.data) >= 4:
            with self._lock:
                for i in range(4):
                    self.pwm_actual[i] = float(msg.data[i])

    def _callback_reset_modelo(self, msg: Bool):
        """Resetea el estado del modelo al origen (llamado por rutina_demo al reiniciar)."""
        if msg.data:
            self.get_logger().info("Reset de modelo solicitado — volviendo al origen")
            self.modelo.reset()
            self.estado = self.modelo.get_estado()
            self.t_anterior = None  # Forzar recalculo de dt en el siguiente paso

    # ── HILO MAVLINK ──────────────────────────────────────────────────────────

    def _bucle_mavlink(self):
        """
        Corre en un hilo separado para no bloquear el timer de ROS 2.
        Lee SERVO_OUTPUT_RAW y actualiza self.pwm_actual continuamente.
        """
        modo = self.params["mavlink_modo"]
        if modo == "udp":
            cadena = f"udpin:{self.params['mavlink_udp_ip']}:{self.params['mavlink_udp_puerto']}"
        else:
            cadena = f"{self.params['mavlink_puerto']},115200"

        self.get_logger().info(f"Conectando MAVLink → {cadena}")

        try:
            self.conexion_mav = mavutil.mavlink_connection(cadena)

            self.get_logger().info("Esperando heartbeat del Pixhawk...")
            hb = self.conexion_mav.wait_heartbeat(timeout=15)
            if hb is None:
                self.get_logger().error("No se recibió heartbeat. Verifica la conexión.")
                return

            self.get_logger().info(
                f"Pixhawk conectado — System ID: {self.conexion_mav.target_system}"
            )

            self._solicitar_servo_stream()

            # Bucle de lectura continua
            while rclpy.ok():
                msg = self.conexion_mav.recv_match(
                    type    = "SERVO_OUTPUT_RAW",
                    blocking= True,
                    timeout = 1.0,
                )
                if msg is not None:
                    with self._lock:
                        for i, canal in enumerate(CANALES_MOTOR):
                            attr = f"servo{canal}_raw"
                            self.pwm_actual[i] = float(getattr(msg, attr, 1000) or 1000)

        except Exception as e:
            self.get_logger().error(f"Error en hilo MAVLink: {e}")

    def _solicitar_servo_stream(self):
        """Pide al Pixhawk que transmita SERVO_OUTPUT_RAW a 20 Hz."""
        try:
            self.conexion_mav.mav.request_data_stream_send(
                self.conexion_mav.target_system,
                self.conexion_mav.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_RAW_CONTROLLER,
                20,  # Hz
                1,   # activar
            )
            self.get_logger().info("Stream SERVO_OUTPUT_RAW solicitado a 20 Hz.")
        except Exception as e:
            self.get_logger().warn(f"No se pudo solicitar stream: {e}")

    # ── CALLBACK PRINCIPAL (timer ROS 2) ─────────────────────────────────────

    def _callback_publicar(self):
        """
        Se ejecuta a la frecuencia definida en el YAML.
        Toma el último PWM disponible, actualiza el modelo y publica.
        """
        # Leer PWM de forma thread-safe
        with self._lock:
            pwm = list(self.pwm_actual)

        # dt adaptativo con clamping de seguridad
        ahora = time.time()
        if self.t_anterior is None:
            dt = self.dt_nominal
        else:
            dt = ahora - self.t_anterior
            if dt <= 0 or dt > 0.5:
                dt = self.dt_nominal
        self.t_anterior = ahora

        # PWM -> omega (rad/s) usando OMEGA_MAX dinámico configurado
        # Mapea PWM [1000, 2000] a [0, omega_max]
        omegas = [
            float(np.clip((p - 1000.0) / 1000.0, 0.0, 1.0) * self.omega_max)
            for p in pwm
        ]
        omega1, omega2, omega3, omega4 = omegas

        # Actualizar modelo con RK4
        try:
            self.estado = self.modelo.actualizar(omega1, omega2, omega3, omega4, dt)
        except Exception as e:
            self.get_logger().warn(f"Error en modelo: {e}")
            return

        # ── Detección de NaN — protección extra en el nodo ────────────────
        if self.modelo.tiene_nan():
            self._nan_reset_count += 1
            self.get_logger().warn(
                f"WARNING: NaN detectado en estado — reseteando modelo "
                f"(reset #{self._nan_reset_count})"
            )
            self.modelo.reset()
            self.estado = self.modelo.get_estado()
            return

        # Publicar pose en ROS 2 (y opcionalmente a Gazebo)
        self._publicar_pose()

        # Publicar velocidades de motores
        self._publicar_motores(omegas, pwm)

    # ── PUBLICADORES ─────────────────────────────────────────────────────────

    def _publicar_pose(self):
        """Publica PoseStamped con posición y orientación del drone."""
        e = self.estado

        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"

        # Posición [m]
        msg.pose.position.x = float(e[0])
        msg.pose.position.y = float(e[1])
        msg.pose.position.z = float(e[2])

        # Orientación: convertir RPY (rad) -> quaternion
        roll, pitch, yaw = float(e[3]), float(e[4]), float(e[5])
        q = quaternion_from_euler(roll, pitch, yaw)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]

        self.pub_pose.publish(msg)

        # Enviar estado a Gazebo si está disponible (asíncrono)
        if self.cli_gazebo is not None and self.cli_gazebo.wait_for_service(timeout_sec=0):
            req = SetEntityState.Request()
            req.state.name = "drone_demo"
            req.state.pose = msg.pose
            req.state.reference_frame = "world"
            self.cli_gazebo.call_async(req)

    def _publicar_motores(self, omegas: list, pwm: list):
        """
        Publica velocidades angulares de los 4 motores.

        Formato del array: [omega1, omega2, omega3, omega4,
                            pwm1,   pwm2,   pwm3,   pwm4]
        """
        msg = Float32MultiArray()
        msg.data = [float(o) for o in omegas] + [float(p) for p in pwm]

        dim = MultiArrayDimension()
        dim.label  = "motores"
        dim.size   = len(msg.data)
        dim.stride = len(msg.data)
        msg.layout.dim = [dim]

        self.pub_motores.publish(msg)

    # ── LIMPIEZA ─────────────────────────────────────────────────────────────

    def destroy_node(self):
        self.get_logger().info("Cerrando nodo gemelo digital...")
        if self._nan_reset_count > 0:
            self.get_logger().info(
                f"Resets por NaN durante la sesión: {self._nan_reset_count}"
            )
        super().destroy_node()


# ── ENTRY POINT ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)

    nodo = NodoGemeloDigital()

    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
