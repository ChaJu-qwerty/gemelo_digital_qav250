import sys
import math
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
from .pid_actitud import PIDActitud

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
#   M1 -> Frontal derecho   -> CCW
#   M2 -> Trasero izquierdo -> CCW
#   M3 -> Frontal izquierdo -> CW
#   M4 -> Trasero derecho   -> CW
#
#   El mapeo real depende del cableado físico del dron. Ver motor_map en el YAML.
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

        self.estado     = np.zeros(12)
        self.t_anterior = None
        self.dt_nominal = 1.0 / params["frecuencia_hz"]
        self.params     = params

        # Setpoint de actitud proveniente de ATTITUDE_TARGET (radianes)
        # (solo usado en MODO DEMO con PID)
        self.setpoint_actitud = (0.0, 0.0, 0.0)
        self.yaw_sp_offset = None

        # ── Actitud REAL del Pixhawk (mensaje ATTITUDE) ───────────────────────
        # En MODO REAL, usamos la orientación medida por el IMU del Pixhawk
        # para calcular la dirección de empuje. Esto elimina la necesidad
        # de simular la dinámica rotacional (que era inestable).
        self.actitud_real = None       # (roll, pitch, yaw) en rad
        self.tasa_angular_real = None  # (p, q, r) en rad/s
        self.yaw_real_offset = None    # Para normalizar yaw inicial a 0

        # Controlador PID de actitud virtual (solo para MODO DEMO)
        pid_fallback = {
            "rollrate_k":  params.get("pid_rollrate_k",  1.0),
            "rollrate_p":  params.get("pid_rollrate_p",  0.15),
            "rollrate_i":  params.get("pid_rollrate_i",  0.20),
            "rollrate_d":  params.get("pid_rollrate_d",  0.003),
            "pitchrate_k": params.get("pid_pitchrate_k", 1.0),
            "pitchrate_p": params.get("pid_pitchrate_p", 0.15),
            "pitchrate_i": params.get("pid_pitchrate_i",  0.20),
            "pitchrate_d": params.get("pid_pitchrate_d", 0.003),
            "yawrate_k":   params.get("pid_yawrate_k",   1.0),
            "yawrate_p":   params.get("pid_yawrate_p",   0.20),
            "yawrate_i":   params.get("pid_yawrate_i",   0.10),
            "yawrate_d":   params.get("pid_yawrate_d",   0.0),
        }
        self.pid_actitud = PIDActitud(fallback=pid_fallback)
        self.get_logger().info("PID de actitud (MODO DEMO) inicializado con fallback.")

        # Calcular OMEGA_MAX dinámicamente según los parámetros del motor del YAML
        motor_kv = self.params.get("motor_kv", 2300.0)
        bateria_voltaje = self.params.get("bateria_voltaje", 14.8)
        esc_eficiencia = self.params.get("esc_eficiencia", 0.85)
        rpm_max = motor_kv * bateria_voltaje * esc_eficiencia
        self.omega_max = rpm_max * (2.0 * np.pi / 60.0)

        # ── Mapeo de canales Pixhawk -> motores del modelo ────────────────────
        # motor_map[i] indica que canal Pixhawk (1-indexado) alimenta al motor
        # Mi+1 del modelo. Ejemplo: [1,2,3,4] = canal 1->M1, canal 2->M2, etc.
        # Si el dron se mueve en el eje incorrecto, cambiar este mapeo en el YAML.
        raw_map = params.get("motor_map", [1, 2, 3, 4])
        self.motor_map = [int(ch) - 1 for ch in raw_map]  # Convertir a 0-indexado
        self.get_logger().info(f"Mapeo de motores (canal Pixhawk -> modelo): {raw_map}")

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
            self.get_logger().info("  🎮 MODO DEMO ACTIVADO — Sin conexión MAVLink")
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
        self.declare_parameter("motor_map",          [1, 2, 3, 4])
        # PID fallback gains
        self.declare_parameter("pid_rollrate_k",     1.0)
        self.declare_parameter("pid_rollrate_p",     0.15)
        self.declare_parameter("pid_rollrate_i",     0.20)
        self.declare_parameter("pid_rollrate_d",     0.003)
        self.declare_parameter("pid_pitchrate_k",    1.0)
        self.declare_parameter("pid_pitchrate_p",    0.15)
        self.declare_parameter("pid_pitchrate_i",    0.20)
        self.declare_parameter("pid_pitchrate_d",    0.003)
        self.declare_parameter("pid_yawrate_k",      1.0)
        self.declare_parameter("pid_yawrate_p",      0.20)
        self.declare_parameter("pid_yawrate_i",      0.10)
        self.declare_parameter("pid_yawrate_d",      0.0)

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
            "motor_map"         : self.get_parameter("motor_map").value,
            # PID fallback gains
            "pid_rollrate_k"    : self.get_parameter("pid_rollrate_k").value,
            "pid_rollrate_p"    : self.get_parameter("pid_rollrate_p").value,
            "pid_rollrate_i"    : self.get_parameter("pid_rollrate_i").value,
            "pid_rollrate_d"    : self.get_parameter("pid_rollrate_d").value,
            "pid_pitchrate_k"   : self.get_parameter("pid_pitchrate_k").value,
            "pid_pitchrate_p"   : self.get_parameter("pid_pitchrate_p").value,
            "pid_pitchrate_i"   : self.get_parameter("pid_pitchrate_i").value,
            "pid_pitchrate_d"   : self.get_parameter("pid_pitchrate_d").value,
            "pid_yawrate_k"     : self.get_parameter("pid_yawrate_k").value,
            "pid_yawrate_p"     : self.get_parameter("pid_yawrate_p").value,
            "pid_yawrate_i"     : self.get_parameter("pid_yawrate_i").value,
            "pid_yawrate_d"     : self.get_parameter("pid_yawrate_d").value,
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
            self.get_logger().info("🔄 Reset de modelo solicitado — volviendo al origen")
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
                    type    = ["SERVO_OUTPUT_RAW", "ATTITUDE", "ATTITUDE_TARGET", "PARAM_VALUE"],
                    blocking= True,
                    timeout = 1.0,
                )
                if msg is None:
                    continue
                tipo = msg.get_type()
                if tipo == "SERVO_OUTPUT_RAW":
                    with self._lock:
                        for i, canal in enumerate(CANALES_MOTOR):
                            attr = f"servo{canal}_raw"
                            self.pwm_actual[i] = float(getattr(msg, attr, 1000) or 1000)

                elif tipo == "ATTITUDE":
                    # ── ACTITUD REAL del IMU del Pixhawk ──────────────────
                    # Estos son los ángulos MEDIDOS, no setpoints.
                    # Los usamos directamente para la dirección de empuje.
                    roll_real  = float(msg.roll)
                    pitch_real = float(msg.pitch)
                    yaw_raw    = float(msg.yaw)

                    with self._lock:
                        # Offset de yaw para que el gemelo empiece en 0
                        if self.yaw_real_offset is None:
                            self.yaw_real_offset = yaw_raw

                        yaw_rel = yaw_raw - self.yaw_real_offset
                        while yaw_rel > math.pi: yaw_rel -= 2.0 * math.pi
                        while yaw_rel < -math.pi: yaw_rel += 2.0 * math.pi

                        self.actitud_real = (roll_real, pitch_real, yaw_rel)
                        self.tasa_angular_real = (
                            float(msg.rollspeed),
                            float(msg.pitchspeed),
                            float(msg.yawspeed),
                        )

                elif tipo == "ATTITUDE_TARGET":
                    # Solo para MODO DEMO fallback
                    q = msg.q
                    roll_sp, pitch_sp, yaw_sp = self._quat_a_euler(
                        float(q[0]), float(q[1]), float(q[2]), float(q[3])
                    )
                    with self._lock:
                        if self.yaw_sp_offset is None:
                            self.yaw_sp_offset = yaw_sp
                        yaw_rel = yaw_sp - self.yaw_sp_offset
                        while yaw_rel > math.pi: yaw_rel -= 2.0 * math.pi
                        while yaw_rel < -math.pi: yaw_rel += 2.0 * math.pi
                        self.setpoint_actitud = (roll_sp, pitch_sp, yaw_rel)

                elif tipo == "PARAM_VALUE":
                    nombre = msg.param_id.rstrip("\x00")
                    valor  = float(msg.param_value)
                    aplicado = self.pid_actitud.actualizar_ganancias(nombre, valor)
                    if aplicado:
                        self.get_logger().info(
                            f"[AUTOTUNE] Ganancia actualizada: {nombre} = {valor:.6f}"
                        )

        except Exception as e:
            self.get_logger().error(f"Error en hilo MAVLink: {e}")

    def _solicitar_servo_stream(self):
        """Pide al Pixhawk SERVO_OUTPUT_RAW (throttle) y ATTITUDE_TARGET (setpoint)."""
        try:
            self.conexion_mav.mav.request_data_stream_send(
                self.conexion_mav.target_system,
                self.conexion_mav.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_RAW_CONTROLLER,
                20, 1,
            )
            self.conexion_mav.mav.request_data_stream_send(
                self.conexion_mav.target_system,
                self.conexion_mav.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
                20, 1,
            )
            self.get_logger().info("Streams SERVO_OUTPUT_RAW y ATTITUDE_TARGET solicitados a 20 Hz.")
            
            # Solicitar ganancias PID al iniciar
            self._solicitar_parametros_pid()
        except Exception as e:
            self.get_logger().warn(f"No se pudo solicitar stream: {e}")

    def _solicitar_parametros_pid(self):
        """Solicita individualmente al Pixhawk los parametros del rate controller."""
        params_a_pedir = [
            "MC_ROLLRATE_K",  "MC_ROLLRATE_P",  "MC_ROLLRATE_I",  "MC_ROLLRATE_D",
            "MC_PITCHRATE_K", "MC_PITCHRATE_P", "MC_PITCHRATE_I", "MC_PITCHRATE_D",
            "MC_YAWRATE_K",   "MC_YAWRATE_P",   "MC_YAWRATE_I",   "MC_YAWRATE_D",
        ]
        self.get_logger().info("Solicitando ganancias PID (MC_*) al Pixhawk...")
        for p in params_a_pedir:
            try:
                self.conexion_mav.mav.param_request_read_send(
                    self.conexion_mav.target_system,
                    self.conexion_mav.target_component,
                    p.encode("utf-8"),
                    -1
                )
                time.sleep(0.01)  # Pequeño delay para no saturar el enlace
            except Exception as e:
                self.get_logger().warn(f"Error solicitando {p}: {e}")

    @staticmethod
    def _quat_a_euler(qw: float, qx: float, qy: float, qz: float) -> tuple:
        """Convierte cuaternion MAVLink (w,x,y,z) a Euler (roll, pitch, yaw) en radianes."""
        sinr = 2.0 * (qw * qx + qy * qz)
        cosr = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = math.atan2(sinr, cosr)
        sinp = 2.0 * (qw * qy - qz * qx)
        pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
        siny = 2.0 * (qw * qz + qx * qy)
        cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw  = math.atan2(siny, cosy)
        return roll, pitch, yaw

    def _mixer_inverso(self, T: float, tau_roll: float,
                       tau_pitch: float, tau_yaw: float) -> tuple:
        """
        Mixer inverso de Luukkonen frame-X con desaturacion.
        Convierte (T, tau_roll, tau_pitch, tau_yaw) -> (omega1..4) [rad/s].

        Resuelve el sistema lineal 4x4 garantizando que la suma de s_i 
        siempre coincida exactamente con A para preservar el empuje.
        """
        k = self.modelo.k
        b = self.modelo.b
        L = self.modelo.l / math.sqrt(2.0)

        A = T / k
        B = tau_roll  / (k * L)
        C = tau_pitch / (k * L)
        D = tau_yaw   / b

        # ── Desaturación del mixer para prevenir tirones de empuje ──
        # Si la suma de las demandas de actitud excede el empuje base A,
        # reducimos B, C, y D proporcionalmente. Esto garantiza que 
        # s1..s4 nunca sean negativos y T se mantenga exactamente igual.
        suma_actitud = abs(B) + abs(C) + abs(D)
        if suma_actitud > A:
            if A > 1e-6:
                escala = A / suma_actitud
                B *= escala
                C *= escala
                D *= escala
            else:
                B = C = D = 0.0

        s1 = max(0.0, (A - B - C - D) / 4.0)
        s2 = max(0.0, (A + B + C - D) / 4.0)
        s3 = max(0.0, (A + B - C + D) / 4.0)
        s4 = max(0.0, (A - B + C + D) / 4.0)

        return math.sqrt(s1), math.sqrt(s2), math.sqrt(s3), math.sqrt(s4)

    # ── CALLBACK PRINCIPAL (timer ROS 2) ─────────────────────────────────────

    def _callback_publicar(self):
        """
        Se ejecuta a la frecuencia definida en el YAML.

        MODO DEMO: usa los PWM de la rutina_demo + modelo Euler-Lagrange completo (RK4).
        MODO REAL: usa empuje de PWM + actitud REAL del Pixhawk (ATTITUDE)
                   para integrar solo la traslacion (X, Y, Z).
        """
        with self._lock:
            pwm      = list(self.pwm_actual)
            setpoint = tuple(self.setpoint_actitud)

        # dt adaptativo
        ahora = time.time()
        if self.t_anterior is None:
            dt = self.dt_nominal
        else:
            dt = ahora - self.t_anterior
            if dt <= 0 or dt > 0.5:
                dt = self.dt_nominal
        self.t_anterior = ahora

        if self.modo_demo:
            # ── MODO DEMO: modelo completo Euler-Lagrange con RK4 ──────────
            omegas = [
                float(np.clip((p - 1000.0) / 1000.0, 0.0, 1.0) * self.omega_max)
                for p in pwm
            ]
            try:
                omega1 = omegas[self.motor_map[0]]
                omega2 = omegas[self.motor_map[1]]
                omega3 = omegas[self.motor_map[2]]
                omega4 = omegas[self.motor_map[3]]
            except IndexError:
                omega1, omega2, omega3, omega4 = omegas

            # Usar modelo RK4 completo (rotacional + translacional)
            try:
                self.estado = self.modelo.actualizar(omega1, omega2, omega3, omega4, dt)
            except Exception as e:
                self.get_logger().warn(f"Error en modelo: {e}")
                return

            if self.modelo.tiene_nan():
                self._nan_reset_count += 1
                self.get_logger().warn(
                    f"WARNING: NaN en estado — reseteando (reset #{self._nan_reset_count})"
                )
                self.modelo.reset()
                self.pid_actitud.reset()
                self.estado = self.modelo.get_estado()
                return

<<<<<<< Updated upstream
        # ── Detección de NaN — protección extra en el nodo ────────────────
        if self.modelo.tiene_nan():
            self._nan_reset_count += 1
            self.get_logger().warn(
                f"⚠️ NaN detectado en estado — reseteando modelo "
                f"(reset #{self._nan_reset_count})"
            )
            self.modelo.reset()
            self.estado = self.modelo.get_estado()
=======
            omegas_pub = [omega1, omega2, omega3, omega4]
            self._publicar_pose()
            self._publicar_motores(omegas_pub, pwm)
>>>>>>> Stashed changes
            return
        else:
            # ── MODO REAL: Actitud real del Pixhawk + PWM directo ─────────
            # Arquitectura correcta:
            #   - El Pixhawk ya hizo toda la cascada de control internamente
            #     (Posicion → Velocidad → Actitud → Rate → Motores)
            #   - Nosotros recibimos el RESULTADO: los 4 PWM y la actitud medida
            #   - Solo necesitamos calcular la traslación (X, Y, Z)
            #     usando el empuje total + la orientación real
            #
            # ¿Por qué NO simulamos la dinámica rotacional?
            #   - Las diferencias mínimas entre PWMs de motores generaban
            #     torques que, sin el lazo de control completo del Pixhawk,
            #     se acumulaban y hacían explotar los ángulos.
            #   - El Pixhawk ya mide la actitud con su IMU (giroscopio +
            #     acelerómetro + magnetómetro). Es mucho más preciso que
            #     nuestra simulación.

            # 1. Convertir PWM → omega para cada motor
            omegas_raw = [
                float(np.clip((p - 1000.0) / 1000.0, 0.0, 1.0) * self.omega_max)
                for p in pwm
            ]
            try:
                omega1 = omegas_raw[self.motor_map[0]]
                omega2 = omegas_raw[self.motor_map[1]]
                omega3 = omegas_raw[self.motor_map[2]]
                omega4 = omegas_raw[self.motor_map[3]]
            except IndexError:
                omega1, omega2, omega3, omega4 = omegas_raw

            # 2. Empuje total T = k * Σ(ωi²)
            T = self.modelo.k * (omega1**2 + omega2**2 + omega3**2 + omega4**2)

            # 3. Obtener actitud real del Pixhawk (con lock)
            with self._lock:
                actitud = self.actitud_real
                tasas   = self.tasa_angular_real

            # 4. Si aún no llega ATTITUDE, usar ángulos = 0 (horizontal)
            if actitud is not None:
                phi, theta, psi = actitud
            else:
                phi, theta, psi = 0.0, 0.0, 0.0

            # 5. Bloqueo en suelo: si empuje < peso y Z <= 0
            peso = self.modelo.m * self.modelo.g
            if T <= peso and float(self.estado[2]) <= 0.0:
                self.estado = np.zeros(12)
                self.estado[3] = phi
                self.estado[4] = theta
                self.estado[5] = psi
                self._publicar_pose()
                self._publicar_motores([omega1, omega2, omega3, omega4], pwm)
                return

            # 6. Dirección de empuje en frame inercial (Luukkonen ec. 21)
            sphi, cphi = np.sin(phi), np.cos(phi)
            stht, ctht = np.sin(theta), np.cos(theta)
            spsi, cpsi = np.sin(psi), np.cos(psi)

            thrust_dir_x = (cpsi * stht * cphi) + (spsi * sphi)
            thrust_dir_y = (spsi * stht * cphi) - (cpsi * sphi)
            thrust_dir_z = ctht * cphi

            # 7. Velocidades actuales del estado
            dx = float(self.estado[6])
            dy = float(self.estado[7])
            dz = float(self.estado[8])

            # 8. Aceleraciones translacionales (Luukkonen ec. 21)
            m = self.modelo.m
            ddx = (T / m) * thrust_dir_x - (self.modelo.Ax / m) * dx
            ddy = (T / m) * thrust_dir_y - (self.modelo.Ay / m) * dy
            ddz = (T / m) * thrust_dir_z - (self.modelo.Az / m) * dz - self.modelo.g

            # 9. Integración semi-implícita (Symplectic Euler)
            dx_new = dx + ddx * dt
            dy_new = dy + ddy * dt
            dz_new = dz + ddz * dt

            x_new = float(self.estado[0]) + dx_new * dt
            y_new = float(self.estado[1]) + dy_new * dt
            z_new = float(self.estado[2]) + dz_new * dt

            # Clamp Z >= 0 (suelo)
            if z_new < 0.0:
                z_new = 0.0
                dz_new = max(0.0, dz_new)

            # 10. Actualizar vector de estado [x,y,z, phi,theta,psi, dx,dy,dz, p,q,r]
            self.estado[0] = x_new
            self.estado[1] = y_new
            self.estado[2] = z_new
            self.estado[3] = phi       # roll real del IMU
            self.estado[4] = theta     # pitch real del IMU
            self.estado[5] = psi       # yaw real del IMU
            self.estado[6] = dx_new
            self.estado[7] = dy_new
            self.estado[8] = dz_new
            if tasas is not None:
                self.estado[9]  = tasas[0]
                self.estado[10] = tasas[1]
                self.estado[11] = tasas[2]

            omegas = [omega1, omega2, omega3, omega4]

        # ── Publicar estado ───────────────────────────────────────────────
        self._publicar_pose()
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
        rclpy.shutdown()


if __name__ == "__main__":
    main()
