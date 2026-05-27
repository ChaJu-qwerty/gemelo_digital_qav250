"""
Esto se va a poder usar de tres maneras: 
QGroundControl
    python captura_pwm.py --modo udp
USB
    python captura_pwm.py --modo serial --puerto COM_N
CSV
    python captura_pwm.py --modo udp --guardar datos_vuelo.csv
"""

import argparse
import csv
import sys
import time
from datetime import datetime
from pathlib import Path
import numpy as np
from pymavlink import mavutil

#Constantes de calibracion
PWM_MIN = 1000
PWM_MAX = 2000

#rs2205 readytosky 2300kv
#checar si esto se puede cambiar midiendo la eficiencia del motor + ESC 
KV_MOTOR = 2300  # RPM/V
V_BATERIA  = 14.8
EFC_ESC    = 0.85
RPM_MAX    = KV_MOTOR * V_BATERIA * EFC_ESC          # ≈ 28,796 RPM
OMEGA_MAX = RPM_MAX *(2.0 * np.pi / 60.0)                # ≈ 3017 rad/s

#canales de salida del pixhawk asignados a cada motor
CANAL_MOTOR = {
    "M1_frontal_der": 1,
    "M2_trasero_izq": 2,
    "M3_frontal_izq": 3,
    "M4_trasero_der": 4,
}

#Configuracion de puertos de conexion 
UDP_IP = "127.0.0.1"
UDP_PUERTO = 14551 # QGC usa 14550; este script escucha en 14551
SERIAL_BAUD = 115200
TIMEOUT_HB      = 10        # segundos para esperar heartbeat
INTERVALO_LOG   = 0.05      # segundos entre lecturas (≈20 Hz)

def pwm_a_omega(pwm: float) -> float:  
    #Convierte un valor PWM a velocidad angular (rad/s)
    pwm_clip = np.clip(pwm, PWM_MIN, PWM_MAX)
    t = (pwm_clip - PWM_MIN) / (PWM_MAX - PWM_MIN)
    return float(t * OMEGA_MAX)

def pwm_a_throttle_pct(pwm: float) -> float:
    #Convierte PWM a porcentaje de throttle (0.0 – 100.0)
    pwm_clip = np.clip(pwm, PWM_MIN, PWM_MAX)
    return float((pwm_clip - PWM_MIN) / (PWM_MAX - PWM_MIN) * 100.0)

#Clase para la lectura de los motores
class LectorMotoresPixhawk:
    """
    Conecta el pixhawk via MAVLink (UDP o Serial) y captura los valores PWM de los motores
        udp : escucha paquetes UDP reenviados por QGroundControl
        serial: conexion directa por USB / UART (sin QGC)
    """
    def __init__(
        self,
        modo: str = "udp",
        puerto_serial: str = "COM3",
        udp_ip: str = UDP_IP,
        udp_puerto: int = UDP_PUERTO,
        guardar_csv: str | None = None,
    ):
        self.modo         = modo
        self.puerto_serial = puerto_serial
        self.udp_ip       = udp_ip
        self.udp_puerto   = udp_puerto
        self.guardar_csv  = guardar_csv

        self.conexion    = None
        self.corriendo   = False
        self._csv_file   = None
        self._csv_writer = None

        # Último estado leído
        self.pwm_motores   = {k: 0.0 for k in CANAL_MOTOR}
        self.omega_motores = {k: 0.0 for k in CANAL_MOTOR}
        self.timestamp     = 0.0
    
    def conectar(self) -> bool:
        """
        Establece la conexión MAVLink con el Pixhawk.

        Retorna TRUE si la conexión fue exitosa.
        """
        try:
            if self.modo == "udp":
                cadena = f"udpin:{self.udp_ip}:{self.udp_puerto}"
                print(f"[INFO] Conectando en modo UDP → {cadena}")
                print( "[INFO] Asegurarse que QGroundControl está activo y")
                print(f"       tiene configurado el reenvío UDP a puerto {self.udp_puerto}")
            else:
                cadena = f"{self.puerto_serial},{SERIAL_BAUD}"
                print(f"[INFO] Conectando en modo Serial → {cadena}")

            self.conexion = mavutil.mavlink_connection(cadena)

            print(f"[INFO] Esperando heartbeat del Pixhawk (timeout {TIMEOUT_HB}s)...")
            msg = self.conexion.wait_heartbeat(timeout=TIMEOUT_HB)

            if msg is None:
                print("[ERROR] No se recibió heartbeat. Verificar la conexión.")
                return False

            print(f"[OK]   Pixhawk detectado.")
            print(f"       System ID: {self.conexion.target_system} | "
                  f"Component ID: {self.conexion.target_component}")

            # Solicitar al Pixhawk que transmita SERVO_OUTPUT_RAW a 20 Hz
            self._solicitar_servo_output()
            return True

        except Exception as e:
            print(f"[ERROR] Fallo al conectar: {e}")
            return False
        
    def _solicitar_servo_output(self):
        """
        Envía comando MAVLink para activar el stream de SERVO_OUTPUT_RAW.
        Esto es necesario cuando el Pixhawk no lo transmite automáticamente.
        """
        try:
            self.conexion.mav.request_data_stream_send(
                self.conexion.target_system,
                self.conexion.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_RAW_CONTROLLER,  # incluye SERVO_OUTPUT_RAW
                20,    # frecuencia en Hz
                1,     # 1 = activar, 0 = desactivar
            )
            print("[INFO] Stream SERVO_OUTPUT_RAW solicitado a 20 Hz")
        except Exception as e:
            print(f"[WARN] No se pudo solicitar stream: {e}")

    def _iniciar_csv(self):
        """Prepara el archivo CSV para guardar los datos capturados."""
        if not self.guardar_csv:
            return
        ruta = Path(self.guardar_csv)
        self._csv_file   = open(ruta, "w", newline="", encoding="utf-8")
        self._csv_writer = csv.writer(self._csv_file)

        encabezado = ["timestamp_s", "datetime"]
        for nombre in CANAL_MOTOR:
            encabezado += [
                f"{nombre}_pwm_us",
                f"{nombre}_throttle_pct",
                f"{nombre}_omega_rad_s",
            ]
        self._csv_writer.writerow(encabezado)
        print(f"[INFO] Guardando datos en: {ruta.resolve()}")

    def _guardar_fila(self):
        """Escribe una fila con el estado actual en el CSV."""
        if not self._csv_writer:
            return
        fila = [
            f"{self.timestamp:.4f}",
            datetime.fromtimestamp(self.timestamp).strftime("%Y-%m-%d %H:%M:%S.%f"),
        ]
        for nombre in CANAL_MOTOR:
            pwm   = self.pwm_motores[nombre]
            pct   = pwm_a_throttle_pct(pwm)
            omega = self.omega_motores[nombre]
            fila += [f"{pwm:.1f}", f"{pct:.2f}", f"{omega:.3f}"]
        self._csv_writer.writerow(fila)

    def _cerrar_csv(self):
        if self._csv_file:
            self._csv_file.flush()
            self._csv_file.close()
            print(f"[INFO] CSV guardado y cerrado.")

    def _procesar_servo_output(self, msg) -> bool:
        """
        Extrae PWM de los 4 motores del mensaje SERVO_OUTPUT_RAW.

        Retorna True si el mensaje contiene datos válidos.
        """
        if msg is None:
            return False

        self.timestamp = time.time()

        for nombre, canal in CANAL_MOTOR.items():
            # Los atributos son servo1_raw, servo2_raw, ... servo8_raw
            attr = f"servo{canal}_raw"
            pwm  = getattr(msg, attr, 0) or 0.0

            self.pwm_motores[nombre]   = float(pwm)
            self.omega_motores[nombre] = pwm_a_omega(pwm)

        return True
    
    def _imprimir_estado(self):
        """Imprime el estado actual de los 4 motores en consola."""
        print("\033[H\033[J", end="")  
        ahora = datetime.fromtimestamp(self.timestamp).strftime("%H:%M:%S.%f")[:-3]
        print("=" * 65)
        print(f"  CAPTURA MOTORES QAV250  |  {ahora}  |  Ctrl+C para salir")
        print("=" * 65)
        print(f"  {'Motor':<22} {'PWM [µs]':>10} {'Throttle %':>12} {'ω [rad/s]':>12}")
        print("-" * 65)
        for nombre in CANAL_MOTOR:
            pwm   = self.pwm_motores[nombre]
            pct   = pwm_a_throttle_pct(pwm)
            omega = self.omega_motores[nombre]
            # Barra visual de throttle
            barras = int(pct / 5)
            barra  = "█" * barras + "░" * (20 - barras)
            print(f"  {nombre:<22} {pwm:>10.0f} {pct:>11.1f}% {omega:>12.1f}")
            print(f"  {'':22} [{barra}]")
        print("=" * 65)
        if self.guardar_csv:
            print(f" Guardando en: {self.guardar_csv}")

    def capturar(self):
        """
        Bucle principal de captura

        lee mensajes SERVO_OUTPUT_RAW y actualiza el estado de los motores
        en tiempo real
        """
        if not self.conectar():
            sys.exit(1)

        self._iniciar_csv()
        self.corriendo = True

        print("[INFO] Iniciando captura... (Ctrl+C para detener)\n")
        time.sleep(1)

        try:
            while self.corriendo:
                t_inicio = time.time()

                # Esperar mensaje SERVO_OUTPUT_RAW con timeout de 1s
                msg = self.conexion.recv_match(
                    type="SERVO_OUTPUT_RAW",
                    blocking=True,
                    timeout=1.0,
                )

                if msg is not None:
                    if self._procesar_servo_output(msg):
                        self._imprimir_estado()
                        self._guardar_fila()
                else:
                    print("[WARN] Sin datos SERVO_OUTPUT_RAW — esperando...", end="\r")

                # Mantener frecuencia de muestreo
                transcurrido = time.time() - t_inicio
                pausa = INTERVALO_LOG - transcurrido
                if pausa > 0:
                    time.sleep(pausa)

        except KeyboardInterrupt:
            print("\n\n[INFO] Captura detenida por el usuario.")
        except Exception as e:
            print(f"\n[ERROR] Error durante la captura: {e}")
            raise
        finally:
            self.corriendo = False
            self._cerrar_csv()
            print("[INFO] Conexión cerrada.")

    def obtener_pwm(self) -> dict:
        #Retorna el último estado PWM leído (para integración con otros módulos).
        return dict(self.pwm_motores)

    def obtener_omega(self) -> dict:
        #Retorna la última velocidad angular calculada (rad/s).
        return dict(self.omega_motores)
    
def crear_lector(modo="udp", puerto="COM3", csv=None) -> LectorMotoresPixhawk:
    """
    Función de conveniencia para crear el lector desde otro módulo / script

    Ejemplo de uso externo:    
    
    from captura_pwm import crear_lector, pwm_a_omega

    lector = crear_lector(modo="udp")
    lector.conectar()

    while True:
        msg = lector.conexion.recv_match(type="SERVO_OUTPUT_RAW", blocking=True, timeout=1)
        if msg:
            lector._procesar_servo_output(msg)
            pwm   = lector.obtener_pwm()
            omega = lector.obtener_omega()
            # → publicar en ROS 2 / Gazebo aquí
    """
    return LectorMotoresPixhawk(modo=modo, puerto_serial=puerto, guardar_csv=csv)

#Inputs por linea de comando
def _parsear_args():
    parser = argparse.ArgumentParser(
        description="Captura PWM de 4 motores del QAV250 desde Pixhawk vía MAVLink",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Ejemplos:
  Con QGroundControl activo (modo UDP recomendado):
    python captura_pwm.py --modo udp

  Conexión directa por USB (Windows):
    python captura_pwm.py --modo serial --puerto COM3

  Conexión directa por USB (Linux):
    python captura_pwm.py --modo serial --puerto /dev/ttyUSB0

  Guardar datos en CSV:
    python captura_pwm.py --modo udp --guardar datos_vuelo.csv

Configurar QGroundControl para reenviar UDP:
  Application Settings -> Comm Links -> Add -> UDP
  Listening Port: 14550  |  Target: 127.0.0.1:14551
        """,
    )
    parser.add_argument(
        "--modo",
        choices=["udp", "serial"],
        default="udp",
        help="Modo de conexión: 'udp' (con QGC) o 'serial' (directo). Default: udp",
    )
    parser.add_argument(
        "--puerto",
        default="COM3",
        help="Puerto serial. Solo aplica con --modo serial. Default: COM3",
    )
    parser.add_argument(
        "--udp-ip",
        default=UDP_IP,
        help=f"IP para escuchar UDP. Default: {UDP_IP}",
    )
    parser.add_argument(
        "--udp-puerto",
        type=int,
        default=UDP_PUERTO,
        help=f"Puerto UDP. Default: {UDP_PUERTO}",
    )
    parser.add_argument(
        "--guardar",
        default=None,
        metavar="ARCHIVO.csv",
        help="Ruta del archivo CSV donde guardar los datos capturados",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = _parsear_args()

    lector = LectorMotoresPixhawk(
        modo          = args.modo,
        puerto_serial = args.puerto,
        udp_ip        = args.udp_ip,
        udp_puerto    = args.udp_puerto,
        guardar_csv   = args.guardar,
    )

    lector.capturar()
