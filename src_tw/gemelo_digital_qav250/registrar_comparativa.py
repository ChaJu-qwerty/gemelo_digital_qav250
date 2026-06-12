"""
Nodo de registro de datos de vuelo comparativo del gemelo digital QAV250.

Suscribe a las poses del dron principal (IMU Pixhawk) y el fantasma (IMU Gyro)
y escribe un log en tiempo real a un archivo .txt para análisis comparativo.

Uso:
    ros2 run gemelo_digital_qav250 registrar_comparativa
"""

import os
import math
import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Vector3


class RegistrarComparativaNode(Node):
    def __init__(self):
        super().__init__("registrar_comparativa")

        # Crear nombre de archivo con timestamp
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self._ruta_log = os.path.join(os.getcwd(), f"comparativa_log_{ts}.txt")

        # Abrir archivo de log
        self._archivo = open(self._ruta_log, "w", buffering=1)
        self._escribir_cabecera()

        # Estado actual
        self._pose_real = None      # (x, y, z, roll, pitch, yaw)
        self._pose_fantasma = None  # (x, y, z, roll, pitch, yaw)
        self._euler_puro = None     # (roll, pitch, yaw)
        self._t0 = None
        self._n_lineas = 0

        # Suscripciones
        self.create_subscription(
            PoseStamped,
            "/qav250/pose",
            self._cb_pose_real,
            10,
        )
        self.create_subscription(
            PoseStamped,
            "/qav250_fantasma/pose",
            self._cb_pose_fantasma,
            10,
        )
        self.create_subscription(
            Vector3,
            "/qav250/euler_calculado",
            self._cb_euler_calculado,
            10,
        )

        self.get_logger().info("=" * 60)
        self.get_logger().info("  REGISTRO COMPARATIVO ACTIVADO")
        self.get_logger().info(f"  Archivo: {self._ruta_log}")
        self.get_logger().info("  Suscribiendo a /qav250/pose y /qav250_fantasma/pose")
        self.get_logger().info("  Presiona Ctrl+C para cerrar y guardar el log.")
        self.get_logger().info("=" * 60)

    # ── CABECERA ──────────────────────────────────────────────────────────────

    def _escribir_cabecera(self):
        self._archivo.write("# Log de vuelo Comparativo (Pixhawk vs Gyro vs Matematico) QAV250\n")
        self._archivo.write(f"# Generado: {datetime.datetime.now().isoformat()}\n")
        self._archivo.write("#\n")
        self._archivo.write("# Columnas:\n")
        self._archivo.write("#   t      = tiempo relativo [s]\n")
        self._archivo.write("#   Xr,Yr,Zr,Roll_r,Pitch_r,Yaw_r = Datos Dron Principal (IMU Pixhawk)\n")
        self._archivo.write("#   Xf,Yf,Zf,Roll_f,Pitch_f,Yaw_f = Datos Dron Fantasma (IMU Stand Gyro)\n")
        self._archivo.write("#   Roll_m,Pitch_m,Yaw_m          = Datos Matematicos (Integracion pura desde Motores)\n")
        self._archivo.write("#\n")
        self._archivo.write(
            f"{'t':>10} | {'Xr':>8} {'Yr':>8} {'Zr':>8} | "
            f"{'Roll_r':>8} {'Pitch_r':>8} {'Yaw_r':>8} | "
            f"{'Xf':>8} {'Yf':>8} {'Zf':>8} | "
            f"{'Roll_f':>8} {'Pitch_f':>8} {'Yaw_f':>8} | "
            f"{'Roll_m':>8} {'Pitch_m':>8} {'Yaw_m':>8}\n"
        )
        self._archivo.write("-" * 170 + "\n")

    # ── CALLBACKS ─────────────────────────────────────────────────────────────

    def _extraer_pose(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

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

        return (
            x, y, z,
            math.degrees(roll),
            math.degrees(pitch),
            math.degrees(yaw),
        )

    def _cb_pose_real(self, msg: PoseStamped):
        self._pose_real = self._extraer_pose(msg)
        
        # El principal rige el tiempo para sincronizar la escritura
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._t0 is None:
            self._t0 = stamp_sec
        t_rel = stamp_sec - self._t0

        self._escribir_linea(t_rel)

    def _cb_pose_fantasma(self, msg: PoseStamped):
        self._pose_fantasma = self._extraer_pose(msg)

    def _cb_euler_calculado(self, msg: Vector3):
        self._euler_puro = (msg.x, msg.y, msg.z)


    # ── ESCRITURA ─────────────────────────────────────────────────────────────

    def _escribir_linea(self, t_rel: float):
        if self._pose_real is None or self._pose_fantasma is None or self._euler_puro is None:
            return

        xr, yr, zr, rollr, pitchr, yawr = self._pose_real
        xf, yf, zf, rollf, pitchf, yawf = self._pose_fantasma
        rollm, pitchm, yawm = self._euler_puro

        linea = (
            f"{t_rel:>10.3f} | "
            f"{xr:>8.3f} {yr:>8.3f} {zr:>8.3f} | "
            f"{rollr:>8.2f} {pitchr:>8.2f} {yawr:>8.2f} | "
            f"{xf:>8.3f} {yf:>8.3f} {zf:>8.3f} | "
            f"{rollf:>8.2f} {pitchf:>8.2f} {yawf:>8.2f} | "
            f"{rollm:>8.2f} {pitchm:>8.2f} {yawm:>8.2f}\n"
        )
        self._archivo.write(linea)
        self._n_lineas += 1

        if self._n_lineas % 20 == 0:
            print(
                f"t={t_rel:7.2f}s | "
                f"Real_Z: {zr:.2f}m | Fant_Z: {zf:.2f}m | "
                f"Real_Roll: {rollr:.1f}° | Fant_Roll: {rollf:.1f}°"
            )

    # ── LIMPIEZA ──────────────────────────────────────────────────────────────

    def destroy_node(self):
        self._archivo.write("-" * 140 + "\n")
        self._archivo.write(f"# Fin de sesion — {self._n_lineas} registros comparativos grabados.\n")
        self._archivo.close()
        self.get_logger().info(f"Log comparativo guardado: {self._ruta_log} ({self._n_lineas} lineas)")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RegistrarComparativaNode()
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
