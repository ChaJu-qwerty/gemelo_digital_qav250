import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np
import os

class VerPoseNode(Node):
    def __init__(self):
        super().__init__('ver_pose')
        self.sub = self.create_subscription(
            PoseStamped,
            '/qav250/pose',
            self.pose_callback,
            10
        )
        self.get_logger().info("Suscribiendo a /qav250/pose. Esperando datos...")

    def pose_callback(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

        # Convertir cuaternión a ángulos de Euler (Roll, Pitch, Yaw)
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)
        else:
            pitch = np.arcsin(sinp)

        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        # Convertir radianes a grados
        r_deg = np.degrees(roll)
        p_deg = np.degrees(pitch)
        y_deg = np.degrees(yaw)

        # Limpiar terminal para que se vea estático y legible
        print("\033[H\033[J", end="")
        print("==================================================")
        print("        TELEMETRÍA DE POSICIÓN Y ORIENTACIÓN      ")
        print("==================================================")
        print("  [UNIDADES DE POSICIÓN: METROS (m)]")
        print(f"    X (Longitud/Pitch): {x:8.3f} m")
        print(f"    Y (Lateral/Roll):    {y:8.3f} m")
        print(f"    Z (Altura/Empuje):  {z:8.3f} m")
        print("--------------------------------------------------")
        print("  [UNIDADES DE ORIENTACIÓN: GRADOS (°)]")
        print(f"    Roll  (Alabeo):     {r_deg:8.2f}°")
        print(f"    Pitch (Cabeceo):    {p_deg:8.2f}°")
        print(f"    Yaw   (Guiñada):    {y_deg:8.2f}°")
        print("==================================================")
        print("  * Presiona Ctrl+C en esta terminal para cerrar.")

def main(args=None):
    rclpy.init(args=args)
    node = VerPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
