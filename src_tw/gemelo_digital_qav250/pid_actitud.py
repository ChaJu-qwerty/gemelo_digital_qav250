"""
Controlador PID de actitud para el gemelo digital QAV250.

Replica la arquitectura estandar del Rate Controller de PX4 (forma ideal):
    Salida = K * (P * error + I * integral - D * d(medicion)/dt)

Las ganancias (K, P, I, D) se pueden recibir en tiempo real via PARAM_VALUE
de MAVLink (parametros MC_ROLLRATE_K/P/I/D, MC_PITCHRATE_K/P/I/D,
MC_YAWRATE_K/P/I/D). Si no llegan, se usan los valores de fallback
cargados del YAML para que el gemelo sea estable desde el arranque.
"""

import math
import numpy as np


class PID:
    """Controlador PID con anti-windup por clamp del integral."""

    def __init__(self, kp: float, ki: float, kd: float,
                 integral_max: float = 1.0, output_max: float = 1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_max = integral_max
        self.output_max   = output_max

        self._integral = 0.0
        self._error_anterior = None

    def actualizar(self, error: float, dt: float) -> float:
        """Calcula la salida del PID dado el error y el paso de tiempo."""
        if dt < 1e-6:
            return 0.0

        # Integral con anti-windup
        self._integral += error * dt
        self._integral = float(np.clip(self._integral,
                                       -self.integral_max, self.integral_max))

        # Derivada (primera iteracion = 0)
        if self._error_anterior is None:
            derivada = 0.0
        else:
            derivada = (error - self._error_anterior) / dt
        self._error_anterior = error

        salida = self.kp * error + self.ki * self._integral + self.kd * derivada
        # La salida de un controlador de vuelo estándar (PX4/Betaflight)
        # está normalizada entre -1.0 y 1.0 antes de ir al mixer.
        return float(np.clip(salida, -self.output_max, self.output_max))

    def reset(self):
        self._integral = 0.0
        self._error_anterior = None


class PIDActitud:
    """
    Tres controladores PID independientes (roll, pitch, yaw).

    Convierte el error entre el setpoint de ATTITUDE_TARGET y la actitud
    actual del modelo en torques de control (tau_roll, tau_pitch, tau_yaw).

    Acepta ganancias de fallback desde el YAML para ser funcional desde el
    arranque. Si llegan PARAM_VALUE via MAVLink, las sobreescriben en tiempo real.
    """

    def __init__(self, fallback: dict = None):
        """
        Args:
            fallback: dict opcional con claves como 'rollrate_k', 'rollrate_p', etc.
                      Estos valores se usan inmediatamente como ganancias iniciales.
        """
        fb = fallback or {}

        # Coeficientes de la forma estandar PX4: Salida = K*(P*err + I*int + D*der)
        self.k_roll  = fb.get("rollrate_k", 0.0)
        self.p_roll  = fb.get("rollrate_p", 0.0)
        self.i_roll  = fb.get("rollrate_i", 0.0)
        self.d_roll  = fb.get("rollrate_d", 0.0)

        self.k_pitch = fb.get("pitchrate_k", 0.0)
        self.p_pitch = fb.get("pitchrate_p", 0.0)
        self.i_pitch = fb.get("pitchrate_i", 0.0)
        self.d_pitch = fb.get("pitchrate_d", 0.0)

        self.k_yaw   = fb.get("yawrate_k", 0.0)
        self.p_yaw   = fb.get("yawrate_p", 0.0)
        self.i_yaw   = fb.get("yawrate_i", 0.0)
        self.d_yaw   = fb.get("yawrate_d", 0.0)

        self.ganancias_recibidas = False  # True cuando al menos 1 PARAM_VALUE llego

        # Crear los PIDs con las ganancias efectivas K*coeficiente
        self.pid_roll  = PID(
            kp=self.k_roll * self.p_roll,
            ki=self.k_roll * self.i_roll,
            kd=self.k_roll * self.d_roll,
        )
        self.pid_pitch = PID(
            kp=self.k_pitch * self.p_pitch,
            ki=self.k_pitch * self.i_pitch,
            kd=self.k_pitch * self.d_pitch,
        )
        self.pid_yaw   = PID(
            kp=self.k_yaw * self.p_yaw,
            ki=self.k_yaw * self.i_yaw,
            kd=self.k_yaw * self.d_yaw,
        )

    def calcular_torques(self,
                         setpoint: tuple,
                         actual: tuple,
                         dt: float) -> tuple:
        """
        Calcula los torques de control.

        Args:
            setpoint: (roll_sp, pitch_sp, yaw_sp) en radianes — de ATTITUDE_TARGET
            actual:   (roll, pitch, yaw) del estado del modelo en radianes
            dt:       paso de tiempo [s]

        Returns:
            (tau_roll, tau_pitch, tau_yaw) en N*m
        """
        roll_sp,  pitch_sp,  yaw_sp  = setpoint
        roll_act, pitch_act, yaw_act = actual

        err_roll  = roll_sp  - roll_act
        err_pitch = pitch_sp - pitch_act
        err_yaw   = self._norm_angulo(yaw_sp - yaw_act)

        tau_roll  = self.pid_roll.actualizar(err_roll,  dt)
        tau_pitch = self.pid_pitch.actualizar(err_pitch, dt)
        tau_yaw   = self.pid_yaw.actualizar(err_yaw,   dt)

        return tau_roll, tau_pitch, tau_yaw

    def actualizar_ganancias(self, param_nombre: str, valor: float):
        """
        Actualiza una ganancia en caliente cuando llega un PARAM_VALUE del Pixhawk.
        Sigue la arquitectura estandar PX4: Ganancia_Efectiva = K * Coeficiente
        """
        # Mapa de nombre PX4 -> (variable_miembro, objeto_pid)
        MAPA = {
            "MC_ROLLRATE_K":  ("k_roll",  self.pid_roll),
            "MC_ROLLRATE_P":  ("p_roll",  self.pid_roll),
            "MC_ROLLRATE_I":  ("i_roll",  self.pid_roll),
            "MC_ROLLRATE_D":  ("d_roll",  self.pid_roll),

            "MC_PITCHRATE_K": ("k_pitch", self.pid_pitch),
            "MC_PITCHRATE_P": ("p_pitch", self.pid_pitch),
            "MC_PITCHRATE_I": ("i_pitch", self.pid_pitch),
            "MC_PITCHRATE_D": ("d_pitch", self.pid_pitch),

            "MC_YAWRATE_K":   ("k_yaw",   self.pid_yaw),
            "MC_YAWRATE_P":   ("p_yaw",   self.pid_yaw),
            "MC_YAWRATE_I":   ("i_yaw",   self.pid_yaw),
            "MC_YAWRATE_D":   ("d_yaw",   self.pid_yaw),
        }
        if param_nombre in MAPA:
            var_nombre, pid_obj = MAPA[param_nombre]
            setattr(self, var_nombre, float(valor))

            # Recalcular ganancias efectivas del PID
            eje = var_nombre.split("_")[1]  # "roll", "pitch", "yaw"
            k = getattr(self, f"k_{eje}")
            p = getattr(self, f"p_{eje}")
            i = getattr(self, f"i_{eje}")
            d = getattr(self, f"d_{eje}")

            pid_obj.kp = k * p
            pid_obj.ki = k * i
            pid_obj.kd = k * d
            self.ganancias_recibidas = True
            return True
        return False

    def reset(self):
        self.pid_roll.reset()
        self.pid_pitch.reset()
        self.pid_yaw.reset()

    @staticmethod
    def _norm_angulo(a: float) -> float:
        """Normaliza un angulo al rango [-pi, pi]."""
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a
