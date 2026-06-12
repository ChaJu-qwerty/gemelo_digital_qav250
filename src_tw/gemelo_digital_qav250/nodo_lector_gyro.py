#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import serial
import re
import math
import threading

class NodoLectorGyro(Node):
    def __init__(self):
        super().__init__('nodo_lector_gyro')
        
        # Parámetros (se pueden sobrescribir desde launch)
        self.declare_parameter("puerto_serial", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 9600)
        
        self.port = self.get_parameter("puerto_serial").value
        self.baud_rate = self.get_parameter("baud_rate").value
        
        # Publicador de actitud del gyro
        self.pub_gyro = self.create_publisher(Vector3, '/qav250/gyro_attitude', 10)
        
        # Expresión regular para parsear (tomada de Gyro.py)
        self.pattern = re.compile(
            rb"z[\x00-\x1f\x2c]{1,5}"           
            rb"([-+]?\d+(?:\.\d+)?)[\x00]*,"    
            rb"([-+]?\d+(?:\.\d+)?)[\x00]*,"    
            rb"([-+]?\d+(?:\.\d+)?)[\x00]*,\{"  
        )
        
        self.thread = threading.Thread(target=self._bucle_lectura)
        self.thread.daemon = True
        self.thread.start()
        
        self.get_logger().info(f"Escuchando Gyro en {self.port} a {self.baud_rate} baudios...")

    def _bucle_lectura(self):
        while rclpy.ok():
            try:
                # Usamos una configuración minimalista como en test_gyro_serial.py
                ser = serial.Serial(self.port, self.baud_rate, timeout=0.5)
                
                buffer = b""
                self.get_logger().info(f"¡Conexión exitosa con Gyro en {self.port}!")
                
                while rclpy.ok():
                    chunk = ser.read(512)
                    if not chunk:
                        continue
                    buffer += chunk
                    matches = list(self.pattern.finditer(buffer))
                    if not matches:
                        if len(buffer) > 1024:
                            buffer = buffer[-512:]
                        continue
                        
                    match = matches[-1]
                    
                    # Parsear los grados crudos del Gyro
                    roll_deg  = float(match.group(1))
                    pitch_deg = float(match.group(2))
                    yaw_deg   = float(match.group(3))

                    # ── CORRECCIÓN: el sensor está montado invertido en el stand ──
                    roll_deg  -= 180.0
                    pitch_deg -= 180.0

                    # Normalizar a [-180, 180]
                    while roll_deg  >  180.0: roll_deg  -= 360.0
                    while roll_deg  < -180.0: roll_deg  += 360.0
                    while pitch_deg >  180.0: pitch_deg -= 360.0
                    while pitch_deg < -180.0: pitch_deg += 360.0
                    while yaw_deg   >  180.0: yaw_deg   -= 360.0
                    while yaw_deg   < -180.0: yaw_deg   += 360.0

                    # ── Calibración de Yaw ──
                    if not hasattr(self, '_yaw_offset_gyro'):
                        self._yaw_offset_gyro = yaw_deg
                        self.get_logger().info(
                            f"Gyro calibrado → Roll={roll_deg:.1f}° "
                            f"Pitch={pitch_deg:.1f}° "
                            f"Yaw offset={yaw_deg:.1f}°"
                        )
                    yaw_deg -= self._yaw_offset_gyro
                    while yaw_deg >  180.0: yaw_deg -= 360.0
                    while yaw_deg < -180.0: yaw_deg += 360.0

                    msg = Vector3()
                    msg.x = math.radians(roll_deg)
                    msg.y = math.radians(pitch_deg)
                    msg.z = math.radians(yaw_deg)

                    self.pub_gyro.publish(msg)
                    
                    # Limpiar buffer
                    ultimo = buffer.rfind(b",{")
                    if ultimo != -1:
                        buffer = buffer[ultimo + 2:]
                    if len(buffer) > 512:
                        buffer = buffer[-256:]
                            
            except Exception as e:
                self.get_logger().error(f"Error conectando al Gyro Serial: {e}. Reintentando en 0.5s...")
                import time
                time.sleep(0.5)

def main(args=None):
    rclpy.init(args=args)
    nodo = NodoLectorGyro()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
