"""
Nodo de registro de datos de vuelo del gemelo digital QAV250.

Suscribe a los topics de pose y motores, y escribe un log en tiempo real
a un archivo .txt con timestamp para analisis posterior.

Uso:
    ros2 run gemelo_digital_qav250 registrar_datos

El archivo de log se guarda en el directorio actual como:
    vuelo_log_YYYYMMDD_HHMMSS.txt

Formato de cada linea:
    t=X.XXX | X=Y.YYY Y=Y.YYY Z=Y.YYY | R=Y.YY P=Y.YY Yaw=Y.YY | w1=YYYY w2=YYYY w3=YYYY w4=YYYY | pwm=[XXXX,XXXX,XXXX,XXXX]
"""

import os
import math
import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray


class RegistrarDatosNode(Node):
    """
    Nodo que graba en un archivo .txt los datos de pose y motores
    en cada instante de tiempo recibido.
    """

    def __init__(self):
        super().__init__("registrar_datos")

        # Crear nombre de archivo con timestamp
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self._ruta_log = os.path.join(os.getcwd(), f"vuelo_log_{ts}.txt")

        # Abrir archivo de log
        self._archivo = open(self._ruta_log, "w", buffering=1)  # buffering=1 = line-buffered
        self._escribir_cabecera()

        # Estado actual
        self._pose_actual   = None   # (x, y, z, roll_deg, pitch_deg, yaw_deg)
        self._motores_actual = None  # (w1, w2, w3, w4, pwm1, pwm2, pwm3, pwm4)
        self._t0            = None   # Tiempo de primer mensaje
        self._n_lineas      = 0

        # Suscripciones
        self.create_subscription(
            PoseStamped,
            "/qav250/pose",
            self._cb_pose,
            10,
        )
        self.create_subscription(
            Float32MultiArray,
            "/qav250/motores",
            self._cb_motores,
            10,
        )

        self.get_logger().info("=" * 60)
        self.get_logger().info("  REGISTRO DE DATOS ACTIVADO")
        self.get_logger().info(f"  Archivo: {self._ruta_log}")
        self.get_logger().info("  Suscribiendo a /qav250/pose y /qav250/motores")
        self.get_logger().info("  Presiona Ctrl+C para cerrar y guardar el log.")
        self.get_logger().info("=" * 60)

    # ── CABECERA ──────────────────────────────────────────────────────────────

    def _escribir_cabecera(self):
        self._archivo.write("# Log de vuelo QAV250 Gemelo Digital\n")
        self._archivo.write(f"# Generado: {datetime.datetime.now().isoformat()}\n")
        self._archivo.write("#\n")
        self._archivo.write("# Columnas:\n")
        self._archivo.write("#   t      = tiempo relativo al primer mensaje [s]\n")
        self._archivo.write("#   X      = posicion longitudinal (Pitch) [m]\n")
        self._archivo.write("#   Y      = posicion lateral (Roll) [m]\n")
        self._archivo.write("#   Z      = altura [m]\n")
        self._archivo.write("#   Roll   = angulo de alabeo [grados]\n")
        self._archivo.write("#   Pitch  = angulo de cabeceo [grados]\n")
        self._archivo.write("#   Yaw    = angulo de guinada [grados]\n")
        self._archivo.write("#   w1..w4 = velocidad angular de cada motor [rad/s]\n")
        self._archivo.write("#   pwm1..4= PWM de cada motor [us]\n")
        self._archivo.write("#\n")
        self._archivo.write(
            f"{'t':>10} | {'X':>8} {'Y':>8} {'Z':>8} | "
            f"{'Roll':>8} {'Pitch':>8} {'Yaw':>8} | "
            f"{'w1':>7} {'w2':>7} {'w3':>7} {'w4':>7} | "
            f"{'pwm1':>5} {'pwm2':>5} {'pwm3':>5} {'pwm4':>5}\n"
        )
        self._archivo.write("-" * 120 + "\n")

    # ── CALLBACKS ─────────────────────────────────────────────────────────────

    def _cb_pose(self, msg: PoseStamped):
        """Recibe PoseStamped y extrae posicion y orientacion."""
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

        # Cuaternion -> Euler (RPY)
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self._pose_actual = (
            x, y, z,
            math.degrees(roll),
            math.degrees(pitch),
            math.degrees(yaw),
        )

        # Obtener tiempo relativo
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._t0 is None:
            self._t0 = stamp_sec
        t_rel = stamp_sec - self._t0

        # Escribir linea al recibit pose (sincronizado con el topic de pose)
        self._escribir_linea(t_rel)

    def _cb_motores(self, msg: Float32MultiArray):
        """Recibe velocidades angulares y PWM de los 4 motores."""
        d = msg.data
        if len(d) >= 8:
            self._motores_actual = tuple(d[:8])
        elif len(d) >= 4:
            self._motores_actual = tuple(d[:4]) + (0.0, 0.0, 0.0, 0.0)

    # ── ESCRITURA ─────────────────────────────────────────────────────────────

    def _escribir_linea(self, t_rel: float):
        """Escribe una linea del log con los datos actuales."""
        if self._pose_actual is None:
            return

        x, y, z, roll, pitch, yaw = self._pose_actual

        if self._motores_actual and len(self._motores_actual) >= 8:
            w1, w2, w3, w4, p1, p2, p3, p4 = self._motores_actual[:8]
        else:
            w1 = w2 = w3 = w4 = 0.0
            p1 = p2 = p3 = p4 = 0.0

        linea = (
            f"{t_rel:>10.3f} | "
            f"{x:>8.3f} {y:>8.3f} {z:>8.3f} | "
            f"{roll:>8.2f} {pitch:>8.2f} {yaw:>8.2f} | "
            f"{w1:>7.1f} {w2:>7.1f} {w3:>7.1f} {w4:>7.1f} | "
            f"{p1:>5.0f} {p2:>5.0f} {p3:>5.0f} {p4:>5.0f}\n"
        )
        self._archivo.write(linea)
        self._n_lineas += 1

        # Imprimir en consola cada 20 lineas (~1 seg a 20 Hz)
        if self._n_lineas % 20 == 0:
            print(
                f"t={t_rel:7.2f}s | "
                f"Pos: ({x:.2f}, {y:.2f}, {z:.2f}) m | "
                f"RPY: ({roll:.1f}, {pitch:.1f}, {yaw:.1f}) deg | "
                f"Lineas: {self._n_lineas}"
            )

    # ── LIMPIEZA ──────────────────────────────────────────────────────────────

    def destroy_node(self):
        self._archivo.write("-" * 120 + "\n")
        self._archivo.write(f"# Fin de sesion — {self._n_lineas} registros grabados.\n")
        self._archivo.write(f"# Cerrado: {datetime.datetime.now().isoformat()}\n")
        self._archivo.close()
        self.get_logger().info(f"Log guardado: {self._ruta_log} ({self._n_lineas} lineas)")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RegistrarDatosNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
