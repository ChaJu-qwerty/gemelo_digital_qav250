#!/usr/bin/env python3
"""
rutina_drone.py
---------------
Spawnea un drone genérico en el world de Hermosillo y lo mueve
por una rutina predeterminada (cuadrado + ascenso) usando
el servicio /gazebo/set_model_state de ROS 2.

Ejecutar CON Gazebo ya corriendo:
    cd ~/ros2_ws/src/gemelo_digital_qav250
    python3 scripts/rutina_drone.py

La rutina:
  1. Despegue: sube de Z=50 a Z=80 en 3 seg
  2. Vuelo cuadrado: 4 lados de 100m cada uno
  3. Regreso al centro
  4. Aterrizaje suave

Ajusta Z_BASE si los edificios están a diferente altura.
"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Twist
import math, time, os

# ─────────────────────────────────────────────
#  CONFIGURACIÓN DE LA RUTINA
# ─────────────────────────────────────────────
Z_BASE      = 50.0    # Altura de despegue en Gazebo (m)
             # Si el terreno del centro está ~30m en Gazebo,
             # pon Z_BASE = 30 + 20 (20m sobre el suelo)
LADO_M      = 150.0   # Lado del cuadrado de vuelo (m)
VEL_MS      = 8.0     # Velocidad de vuelo (m/s)
PASO_SEG    = 0.05    # Intervalo de actualización (seg)
# ─────────────────────────────────────────────

SDF_DRONE = open(
    os.path.join(os.path.dirname(__file__), "..", "models", "drone_demo", "drone_simple.sdf")
).read()


class RutinaDrone(Node):

    def __init__(self):
        super().__init__('rutina_drone')
        self.pub_state = self.create_publisher(
            ModelState, '/gazebo/set_model_state', 10)

        # Waypoints de la rutina (x, y, z)
        z = Z_BASE
        L = LADO_M / 2
        self.waypoints = [
            (  0,   0,  Z_BASE - 20),   # punto inicial (suelo)
            (  0,   0,  Z_BASE),         # despegue
            (  L,   0,  Z_BASE),         # esquina 1
            (  L,   L,  Z_BASE),         # esquina 2
            ( -L,   L,  Z_BASE),         # esquina 3
            ( -L,  -L,  Z_BASE),         # esquina 4
            (  L,  -L,  Z_BASE),         # esquina 5
            (  0,   0,  Z_BASE),         # regreso centro
            (  0,   0,  Z_BASE - 20),   # aterrizaje
        ]
        self.wp_idx = 0
        self.x = 0.0
        self.y = 0.0
        self.z = Z_BASE - 20

        self.get_logger().info(f'Drone iniciado en Z={self.z:.1f}')
        self.get_logger().info(f'Rutina: {len(self.waypoints)} waypoints')
        self.timer = self.create_timer(PASO_SEG, self._step)

    def _mover_hacia(self, tx, ty, tz):
        """Mueve el drone un paso hacia el target. Retorna True si llegó."""
        dx = tx - self.x
        dy = ty - self.y
        dz = tz - self.z
        dist = math.sqrt(dx**2 + dy**2 + dz**2)

        paso = VEL_MS * PASO_SEG
        if dist < paso:
            self.x, self.y, self.z = tx, ty, tz
            return True

        factor = paso / dist
        self.x += dx * factor
        self.y += dy * factor
        self.z += dz * factor
        return False

    def _publicar_pose(self):
        msg = ModelState()
        msg.model_name = 'drone_demo'
        msg.pose.position.x = float(self.x)
        msg.pose.position.y = float(self.y)
        msg.pose.position.z = float(self.z)
        msg.pose.orientation.w = 1.0
        msg.reference_frame = 'world'
        self.pub_state.publish(msg)

    def _step(self):
        if self.wp_idx >= len(self.waypoints):
            self.get_logger().info('Rutina completada')
            self.timer.cancel()
            return

        tx, ty, tz = self.waypoints[self.wp_idx]
        llegue = self._mover_hacia(tx, ty, tz)
        self._publicar_pose()

        if llegue:
            self.get_logger().info(
                f'WP {self.wp_idx+1}/{len(self.waypoints)} alcanzado: '
                f'({tx:.0f}, {ty:.0f}, {tz:.0f})')
            self.wp_idx += 1


def main(args=None):
    rclpy.init(args=args)
    node = RutinaDrone()
    print("\n" + "="*50)
    print("  Drone en rutina automatica")
    print(f"  Z base     : {Z_BASE} m")
    print(f"  Lado cuad  : {LADO_M} m")
    print(f"  Velocidad  : {VEL_MS} m/s")
    print("  Ctrl+C para detener")
    print("="*50 + "\n")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
