"""
Rutina de demostración para probar el gemelo digital SIN drone físico.

Genera secuencias de PWM simuladas que ejercitan los 4 motores en
patrones predefinidos: hover, roll, pitch, yaw, y vuelo combinado.

Uso:
    ros2 run gemelo_digital_qav250 rutina_demo

O mediante el launch file completo:
    ros2 launch gemelo_digital_qav250 gemelo_demo.launch.py
"""

import math
import time
import numpy as np

import rclpy
from rclpy.node import Node
# pyrefly: ignore [missing-import]
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, Bool


# ── Constantes Fijas ──────────────────────────────────────────────────────────
PWM_MIN = 1000.0
PWM_MAX = 2000.0
PWM_IDLE = 1000.0  # Motores apagados

# Delta máximo de perturbación — pequeño para que el dron no se escape
# La demo es para VALIDAR ROTACIONES, no traslaciones grandes
DELTA_ROT = 15.0   # µs de PWM — suficiente para generar torque visible
DELTA_YAW = 12.0   # µs de PWM — yaw necesita menos delta


class RutinaDemoNode(Node):
    """
    Nodo ROS 2 que publica señales PWM simuladas en /qav250/pwm_demo.
    
    El nodo genera automáticamente secuencias de PWM que hacen al gemelo
    digital ejecutar maniobras básicas sin necesidad del Pixhawk.
    """

    def __init__(self):
        super().__init__("rutina_demo")

        self.pub_pwm = self.create_publisher(
            Float32MultiArray,
            "/qav250/pwm_demo",
            10,
        )
        # Topic para pedir reset del estado del modelo al inicio de cada ciclo
        self.pub_reset = self.create_publisher(
            Bool,
            "/qav250/reset_modelo",
            10,
        )

        self.t_inicio = time.time()
        self.fase_idx = 0

        # ── Cargar parámetros físicos para calcular PWM_HOVER dinámicamente ───
        self.declare_parameter("m", 0.650)
        self.declare_parameter("k", 3.13e-5)
        self.declare_parameter("motor_kv", 2300.0)
        self.declare_parameter("bateria_voltaje", 14.8)
        self.declare_parameter("esc_eficiencia", 0.85)
        self.declare_parameter("frecuencia_hz", 20.0)

        m = self.get_parameter("m").value
        k_empuje = self.get_parameter("k").value
        motor_kv = self.get_parameter("motor_kv").value
        bateria_voltaje = self.get_parameter("bateria_voltaje").value
        esc_eficiencia = self.get_parameter("esc_eficiencia").value
        frecuencia_hz = self.get_parameter("frecuencia_hz").value

        # Cálculos de hover dinámicos
        g = 9.81
        omega_hover = np.sqrt((m * g) / (4.0 * k_empuje))  # rad/s
        
        rpm_max = motor_kv * bateria_voltaje * esc_eficiencia
        omega_max = rpm_max * (2.0 * np.pi / 60.0)
        
        self.PWM_HOVER = float(np.clip(
            (omega_hover / omega_max) * (PWM_MAX - PWM_MIN) + PWM_MIN,
            PWM_MIN, PWM_MAX
        ))

        # ── Definición de las fases de demostración ───────────────────────────
        # Cada fase: (nombre, duración_s, función_generadora)
        # NOTA: Los deltas son pequeños a propósito — la demo valida ROTACIONES,
        # no traslaciones. Con deltas grandes el dron se escapa a kilómetros.
        self.fases = [
            ("DESPEGUE SUAVE",    4.0,  self._despegue),
            ("HOVER ESTABLE",     3.0,  self._hover),
            ("ROLL IZQUIERDA",    3.0,  self._roll_izq),
            ("ROLL DERECHA",      3.0,  self._roll_der),
            ("HOVER TRANSICIÓN",  2.0,  self._hover),
            ("PITCH ADELANTE",    3.0,  self._pitch_fwd),
            ("PITCH ATRÁS",       3.0,  self._pitch_bwd),
            ("HOVER TRANSICIÓN",  2.0,  self._hover),
            ("YAW HORARIO",       3.0,  self._yaw_cw),
            ("YAW ANTIHORARIO",   3.0,  self._yaw_ccw),
            ("DESCENSO",          4.0,  self._descenso),
            ("MOTORES APAGADOS",  2.0,  self._apagado),
        ]

        self.t_fase_inicio = time.time()
        self._fase_nombre_actual = ""

        self.timer = self.create_timer(1.0 / frecuencia_hz, self._callback)
        self.get_logger().info("=" * 60)
        self.get_logger().info("  RUTINA DE DEMOSTRACIÓN — Gemelo Digital QAV250")
        self.get_logger().info(f"  Masa configurada: {m:.3f} kg | k: {k_empuje}")
        self.get_logger().info(f"  PWM_HOVER calculado dinámicamente: {self.PWM_HOVER:.1f} µs  "
                               f"(omega_hover={omega_hover:.1f} rad/s)")
        self.get_logger().info("  Publicando PWM simulados en /qav250/pwm_demo")
        self.get_logger().info(f"  Fases: {len(self.fases)} | Duración total: "
                               f"{sum(d for _, d, _ in self.fases):.0f}s")
        self.get_logger().info("=" * 60)

    def _callback(self):
        """Timer callback: genera y publica PWM de la fase actual."""
        if self.fase_idx >= len(self.fases):
            # Reiniciar la rutina — resetear el modelo para volver al origen
            self.get_logger().info("🔄 Reiniciando rutina — reseteando pose al origen...")
            reset_msg = Bool()
            reset_msg.data = True
            self.pub_reset.publish(reset_msg)
            self.fase_idx = 0
            self.t_fase_inicio = time.time()

        nombre, duracion, generador = self.fases[self.fase_idx]
        t_en_fase = time.time() - self.t_fase_inicio

        # Log de cambio de fase
        if nombre != self._fase_nombre_actual:
            self._fase_nombre_actual = nombre
            self.get_logger().info(f"▶ Fase {self.fase_idx + 1}/{len(self.fases)}: "
                                   f"{nombre} ({duracion:.0f}s)")

        # Verificar si la fase terminó
        if t_en_fase >= duracion:
            self.fase_idx += 1
            self.t_fase_inicio = time.time()
            return

        # Generar PWM para esta fase
        progreso = t_en_fase / duracion  # 0.0 → 1.0
        pwm = generador(t_en_fase, progreso)

        # Clamp de seguridad
        pwm = [float(np.clip(p, PWM_MIN, PWM_MAX)) for p in pwm]

        # Publicar
        msg = Float32MultiArray()
        msg.data = pwm
        dim = MultiArrayDimension()
        dim.label = "pwm_demo"
        dim.size = 4
        dim.stride = 4
        msg.layout.dim = [dim]
        self.pub_pwm.publish(msg)

    # ── Generadores de PWM para cada fase ─────────────────────────────────────
    # Todos retornan [pwm_M1, pwm_M2, pwm_M3, pwm_M4]
    # M1: Frontal Derecho  (CCW)
    # M2: Trasero Izquierdo (CCW)
    # M3: Frontal Izquierdo (CW)
    # M4: Trasero Derecho   (CW)

    def _despegue(self, t, progreso):
        """Incremento gradual de throttle de idle a un valor superior a hover para despegar y ganar altura."""
        pwm = PWM_IDLE + ((self.PWM_HOVER + 12.0) - PWM_IDLE) * progreso
        return [pwm, pwm, pwm, pwm]

    def _hover(self, t, progreso):
        """Hover estable — todos los motores iguales."""
        return [self.PWM_HOVER, self.PWM_HOVER, self.PWM_HOVER, self.PWM_HOVER]

    def _roll_izq(self, t, progreso):
        """Roll a la izquierda (tau_phi negativo): Der (M1, M4) sube, Izq (M2, M3) baja."""
        delta = DELTA_ROT * math.sin(math.pi * progreso)  # suave: sube y baja
        return [self.PWM_HOVER + delta, self.PWM_HOVER - delta,
                self.PWM_HOVER - delta, self.PWM_HOVER + delta]

    def _roll_der(self, t, progreso):
        """Roll a la derecha (tau_phi positivo): Izq (M2, M3) sube, Der (M1, M4) baja."""
        delta = DELTA_ROT * math.sin(math.pi * progreso)
        return [self.PWM_HOVER - delta, self.PWM_HOVER + delta,
                self.PWM_HOVER + delta, self.PWM_HOVER - delta]

    def _pitch_fwd(self, t, progreso):
        """Pitch adelante (tau_theta positivo): Traseros (M2, M4) suben, Frontales (M1, M3) bajan.
        
        El aumento en la parte trasera empuja la cola hacia arriba y la nariz hacia abajo.
        """
        delta = DELTA_ROT * math.sin(math.pi * progreso)
        return [self.PWM_HOVER - delta, self.PWM_HOVER + delta,
                self.PWM_HOVER - delta, self.PWM_HOVER + delta]

    def _pitch_bwd(self, t, progreso):
        """Pitch atrás (tau_theta negativo): Frontales (M1, M3) suben, Traseros (M2, M4) bajan."""
        delta = DELTA_ROT * math.sin(math.pi * progreso)
        return [self.PWM_HOVER + delta, self.PWM_HOVER - delta,
                self.PWM_HOVER + delta, self.PWM_HOVER - delta]

    def _yaw_cw(self, t, progreso):
        """Yaw horario (tau_psi negativo): Rotores CCW (M1, M2) suben, CW (M3, M4) bajan.
        
        Por conservación de momento angular, al aumentar la rotación CCW se genera torque de reacción CW.
        """
        delta = DELTA_YAW * math.sin(math.pi * progreso)
        return [self.PWM_HOVER + delta, self.PWM_HOVER + delta,
                self.PWM_HOVER - delta, self.PWM_HOVER - delta]

    def _yaw_ccw(self, t, progreso):
        """Yaw anti-horario (tau_psi positivo): Rotores CW (M3, M4) suben, CCW (M1, M2) bajan."""
        delta = DELTA_YAW * math.sin(math.pi * progreso)
        return [self.PWM_HOVER - delta, self.PWM_HOVER - delta,
                self.PWM_HOVER + delta, self.PWM_HOVER + delta]

    def _descenso(self, t, progreso):
        """Descenso gradual de hover a idle."""
        pwm = self.PWM_HOVER - (self.PWM_HOVER - PWM_IDLE) * progreso
        return [pwm, pwm, pwm, pwm]

    def _apagado(self, t, progreso):
        """Motores apagados."""
        return [PWM_IDLE, PWM_IDLE, PWM_IDLE, PWM_IDLE]


def main(args=None):
    rclpy.init(args=args)
    nodo = RutinaDemoNode()

    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
