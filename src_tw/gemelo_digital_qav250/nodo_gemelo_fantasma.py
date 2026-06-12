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
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, Bool, Float32

try:
    from tf_transformations import quaternion_from_euler
except ImportError:
    print("[ERROR] tf_transformations no encontrado.")
    print("        Instalar: sudo apt install ros-humble-tf-transformations")
    sys.exit(1)


# ── Importar módulos del paquete ──────────────────────────────────────────────
from .modelo_euler_lagrange import DroneModel

try:
    from gazebo_msgs.msg import EntityState
    from gazebo_msgs.srv import SetEntityState
    HAS_GAZEBO_MSGS = True
except ImportError:
    HAS_GAZEBO_MSGS = False


class NodoGemeloFantasma(Node):
    """
    Nodo ROS 2 'fantasma' para la comparativa con el stand de pruebas (FFT Gyro).

    Arquitectura:
      - Recibe los mismos PWM que el dron principal (topic /qav250/pwm_raw)
      - Recibe la actitud del sensor Gyro externo (topic /qav250/gyro_attitude)
      - Integra la traslación usando la actitud del Gyro (en lugar del IMU del Pixhawk)
      - Publica la pose resultante en /qav250_fantasma/pose
      - Mueve la entidad 'drone_fantasma' en Gazebo

    De esta forma, en Gazebo se ven dos drones:
      1. El principal (controlado por el Pixhawk IMU)
      2. El fantasma (controlado por el Gyro del stand)
    """

    def __init__(self):
        super().__init__("nodo_gemelo_fantasma")

        # Declarar y cargar parámetros desde YAML
        self._declarar_parametros()
        params = self._cargar_parametros()

        self.get_logger().info("[FANTASMA] Parámetros cargados desde YAML:")
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
        self.get_logger().info("[FANTASMA] Modelo Euler-Lagrange inicializado.")

        # ── Publishers ────────────────────────────────────────────────────────
        self.topic_pose = params["topic_pose"]
        self.pub_pose = self.create_publisher(
            PoseStamped,
            self.topic_pose,
            10,
        )
        self.pub_motores = self.create_publisher(
            Float32MultiArray,
            params["topic_motores"],
            10,
        )
        self.get_logger().info(f"[FANTASMA] Publicando pose en:    {self.topic_pose}")

        # Service client opcional hacia Gazebo
        if HAS_GAZEBO_MSGS:
            self.cli_gazebo = self.create_client(
                SetEntityState,
                "/set_entity_state"
            )
            self.get_logger().info("[FANTASMA] Gazebo SetEntityState client activo")
        else:
            self.cli_gazebo = None

        self.estado     = np.zeros(12)
        self.t_anterior = None
        self.dt_nominal = 1.0 / params["frecuencia_hz"]
        self.params     = params

        # Actitud del Gyro FFT
        self.actitud_real = None       # (roll, pitch, yaw) en rad
        self.tasa_angular_real = (0.0, 0.0, 0.0)

        # Calcular OMEGA_MAX dinámicamente
        motor_kv = params.get("motor_kv", 2300.0)
        bateria_voltaje = params.get("bateria_voltaje", 14.8)
        esc_eficiencia = params.get("esc_eficiencia", 0.73)
        rpm_max = motor_kv * bateria_voltaje * esc_eficiencia
        self.omega_max = rpm_max * (2.0 * np.pi / 60.0)

        # Mapeo de motores
        raw_map = params.get("motor_map", [1, 2, 3, 4])
        self.motor_map = [int(ch) - 1 for ch in raw_map]

        # PWM actual
        self.pwm_actual = [1000.0] * 4
        self._lock = threading.Lock()

        # ── Suscriptores ──────────────────────────────────────────────────────
        self.sub_pwm = self.create_subscription(
            Float32MultiArray, '/qav250/pwm_raw', self._cb_pwm, 10)
        self.sub_bat = self.create_subscription(
            Float32, '/qav250/battery_voltage', self._cb_bat, 10)
        self.sub_gyro = self.create_subscription(
            Vector3, '/qav250/gyro_attitude', self._cb_gyro, 10)

        self.get_logger().info("[FANTASMA] Suscrito a /qav250/pwm_raw, /qav250/battery_voltage, /qav250/gyro_attitude")

        # ── Timer principal ───────────────────────────────────────────────────
        self.timer = self.create_timer(
            self.dt_nominal,
            self._callback_publicar,
        )
        self.get_logger().info(
            f"[FANTASMA] Nodo listo a {params['frecuencia_hz']} Hz. Esperando datos..."
        )

    # ── CALLBACKS DE DATOS ────────────────────────────────────────────────────

    def _cb_pwm(self, msg):
        with self._lock:
            self.pwm_actual = list(msg.data)

    def _cb_bat(self, msg):
        voltaje = msg.data
        if voltaje > 5.0:
            with self._lock:
                motor_kv = self.params.get("motor_kv", 2300.0)
                esc_eficiencia = self.params.get("esc_eficiencia", 0.73)
                rpm_max = motor_kv * voltaje * esc_eficiencia
                self.omega_max = rpm_max * (2.0 * math.pi / 60.0)

    def _cb_gyro(self, msg):
        # El nodo_lector_gyro ya envía los ángulos en radianes y normalizados a [-π, π].
        # Aplicamos la misma conversión NED→ENU que el nodo principal para consistencia.
        roll_real  = msg.x
        pitch_real = -msg.y   # Inversión NED → ENU
        yaw_raw    = (math.pi / 2.0) - msg.z  # Rotación NED → ENU

        with self._lock:
            yaw_rel = yaw_raw
            while yaw_rel > math.pi: yaw_rel -= 2.0 * math.pi
            while yaw_rel < -math.pi: yaw_rel += 2.0 * math.pi

            self.actitud_real = (roll_real, pitch_real, yaw_rel)

    # ── PARÁMETROS ────────────────────────────────────────────────────────────

    def _declarar_parametros(self):
        """Declarar todos los parámetros que puede recibir del YAML."""
        self.declare_parameter("m",                  0.580)
        self.declare_parameter("l",                  0.1261)
        self.declare_parameter("Ixx",                0.00264)
        self.declare_parameter("Iyy",                0.00220)
        self.declare_parameter("Izz",                0.00420)
        self.declare_parameter("k",                  1.28e-6)
        self.declare_parameter("b",                  1.90e-8)
        self.declare_parameter("Ir",                 6.00e-5)
        self.declare_parameter("Ax",                 0.25)
        self.declare_parameter("Ay",                 0.25)
        self.declare_parameter("Az",                 0.50)
        self.declare_parameter("frecuencia_hz",      20.0)
        self.declare_parameter("topic_pose",         "/qav250_fantasma/pose")
        self.declare_parameter("topic_motores",      "/qav250_fantasma/motores")
        self.declare_parameter("motor_kv",           2300.0)
        self.declare_parameter("bateria_voltaje",    14.8)
        self.declare_parameter("esc_eficiencia",     0.73)
        self.declare_parameter("motor_map",          [1, 2, 3, 4])
        self.declare_parameter("bloquear_xy",        False)
        self.declare_parameter("multiplicador_imu",  1.0)

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
            "frecuencia_hz"     : self.get_parameter("frecuencia_hz").value,
            "topic_pose"        : self.get_parameter("topic_pose").value,
            "topic_motores"     : self.get_parameter("topic_motores").value,
            "motor_kv"          : self.get_parameter("motor_kv").value,
            "bateria_voltaje"   : self.get_parameter("bateria_voltaje").value,
            "esc_eficiencia"    : self.get_parameter("esc_eficiencia").value,
            "motor_map"         : self.get_parameter("motor_map").value,
            "bloquear_xy"       : self.get_parameter("bloquear_xy").value,
        }

    # ── CALLBACK PRINCIPAL (timer ROS 2) ─────────────────────────────────────

    def _callback_publicar(self):
        """
        Integra la traslación usando los PWM del dron principal
        y la actitud del sensor Gyro del stand de pruebas.
        (Misma arquitectura que el modo real del nodo principal,
         pero con la IMU reemplazada por el Gyro.)
        """
        with self._lock:
            pwm = list(self.pwm_actual)

        # dt adaptativo
        ahora = time.time()
        if self.t_anterior is None:
            dt = self.dt_nominal
        else:
            dt = ahora - self.t_anterior
            if dt <= 0 or dt > 0.5:
                dt = self.dt_nominal
        self.t_anterior = ahora

        # 1. Convertir PWM → omega
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

        # 2. Empuje total
        T = self.modelo.k * (omega1**2 + omega2**2 + omega3**2 + omega4**2)

        # 3. Obtener actitud del Gyro
        with self._lock:
            actitud = self.actitud_real
            tasas   = self.tasa_angular_real

        if actitud is not None:
            phi, theta, psi = actitud

            # Multiplicador de IMU (si aplica)
            mult = self.get_parameter("multiplicador_imu").value
            phi *= mult
            theta *= mult

            # Clamp de seguridad
            phi = float(np.clip(phi, -1.48, 1.48))
            theta = float(np.clip(theta, -1.48, 1.48))
        else:
            phi, theta, psi = 0.0, 0.0, 0.0

        # 4. Dirección de empuje en frame inercial (Luukkonen ec. 21)
        # FIX: Forzar psi=0 para la traslación, así Pitch mueve siempre en X y Roll en Y,
        # protegiendo al sistema de posibles derivas del magnetómetro.
        sphi, cphi = np.sin(phi), np.cos(phi)
        stht, ctht = np.sin(theta), np.cos(theta)

        thrust_dir_x = (1.0 * stht * cphi) + (0.0 * sphi)
        thrust_dir_y = (0.0 * stht * cphi) - (1.0 * sphi)
        thrust_dir_z = ctht * cphi

        # 5. Velocidades actuales del estado
        dx = float(self.estado[6])
        dy = float(self.estado[7])
        dz = float(self.estado[8])

        # 6. Aceleraciones
        m = self.modelo.m
        accel_x = (T / m) * thrust_dir_x
        accel_y = (T / m) * thrust_dir_y
        accel_z = (T / m) * thrust_dir_z - self.modelo.g

        # 7. Integración Implícita del Arrastre
        dx_new = (dx + accel_x * dt) / (1.0 + (self.modelo.Ax / m) * dt)
        dy_new = (dy + accel_y * dt) / (1.0 + (self.modelo.Ay / m) * dt)
        dz_new = (dz + accel_z * dt) / (1.0 + (self.modelo.Az / m) * dt)

        # 8. Integración de posición (Symplectic Euler)
        x_new = float(self.estado[0]) + dx_new * dt
        y_new = float(self.estado[1]) + dy_new * dt
        z_new = float(self.estado[2]) + dz_new * dt

        # Clamp Z >= 0 (suelo) y Fricción Estática
        if z_new <= 0.0:
            z_new = 0.0
            dz_new = max(0.0, dz_new)
            dx_new = 0.0
            dy_new = 0.0

        # ── MODO STAND DE PRUEBAS ──
        if self.params.get("bloquear_xy", False):
            x_new = 0.0
            y_new = 0.0
            dx_new = 0.0
            dy_new = 0.0

        # 9. Actualizar estado
        self.estado[0] = x_new
        self.estado[1] = y_new
        self.estado[2] = z_new
        self.estado[3] = phi
        self.estado[4] = theta
        self.estado[5] = psi
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
        """Publica PoseStamped con posición y orientación del drone fantasma."""
        e = self.estado

        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"

        msg.pose.position.x = float(e[0])
        msg.pose.position.y = float(e[1])
        msg.pose.position.z = float(e[2])

        roll, pitch, yaw = float(e[3]), float(e[4]), float(e[5])
        q = quaternion_from_euler(roll, pitch, yaw)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]

        self.pub_pose.publish(msg)

        # Enviar estado a Gazebo — entidad "drone_fantasma" (NO drone_demo)
        if self.cli_gazebo is not None and self.cli_gazebo.wait_for_service(timeout_sec=0):
            req = SetEntityState.Request()
            req.state.name = "drone_fantasma"
            req.state.pose = msg.pose
            req.state.reference_frame = "world"
            self.cli_gazebo.call_async(req)

    def _publicar_motores(self, omegas: list, pwm: list):
        """Publica velocidades angulares de los 4 motores."""
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
        self.get_logger().info("[FANTASMA] Cerrando nodo gemelo fantasma...")
        super().destroy_node()


# ── ENTRY POINT ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)

    nodo = NodoGemeloFantasma()

    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
