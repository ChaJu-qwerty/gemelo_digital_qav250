#!/usr/bin/env python3
"""
generador_terreno_node.py
--------------------------
Nodo ROS 2 que:
  1) Escucha el GPS del Pixhawk via MAVROS (/mavros/global_position/global)
  2) Cuando recibe una posición válida, genera el mapa de Gazebo centrado ahí
  3) Publica el path del .world generado para que otros nodos lo usen

Tópicos suscritos:
    /mavros/global_position/global  (sensor_msgs/NavSatFix)

Tópicos publicados:
    /gemelo/world_path              (std_msgs/String)

Uso:
    ros2 run gemelo_digital_qav250 generador_terreno_node
"""

import os
import sys
import threading
import subprocess

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String


class GeneradorTerrenoNode(Node):

    def __init__(self):
        super().__init__('generador_terreno_node')

        # ── Parámetros declarados (sobreescribibles desde el launch) ──
        self.declare_parameter('radio_km',    3.0)
        self.declare_parameter('resolucion',  129)
        self.declare_parameter('escala_z',    1.0)
        self.declare_parameter('usar_gps',    True)   # False → usa coordenada fija
        self.declare_parameter('lat_fija',    29.0729)
        self.declare_parameter('lon_fija',   -110.9559)
        self.declare_parameter('output_dir',  os.path.expanduser('~/ros2_ws/src/gemelo_digital_qav250'))

        self.radio_km   = self.get_parameter('radio_km').value
        self.resolucion = self.get_parameter('resolucion').value
        self.escala_z   = self.get_parameter('escala_z').value
        self.usar_gps   = self.get_parameter('usar_gps').value
        self.lat_fija   = self.get_parameter('lat_fija').value
        self.lon_fija   = self.get_parameter('lon_fija').value
        self.output_dir = self.get_parameter('output_dir').value

        # Estado interno
        self._generando      = False
        self._world_generado = False

        # Publicador del path del world
        self.pub_world = self.create_publisher(String, '/gemelo/world_path', 10)

        if self.usar_gps:
            self.get_logger().info('🛰️  Esperando GPS del Pixhawk en /mavros/global_position/global ...')
            self.sub_gps = self.create_subscription(
                NavSatFix,
                '/mavros/global_position/global',
                self._callback_gps,
                10
            )
        else:
            self.get_logger().info(f'📍 Usando coordenada fija: ({self.lat_fija}, {self.lon_fija})')
            # Generar inmediatamente con la coordenada fija
            threading.Thread(
                target=self._generar_terreno,
                args=(self.lat_fija, self.lon_fija),
                daemon=True
            ).start()

    def _callback_gps(self, msg: NavSatFix):
        """Callback del GPS — solo genera el mapa la primera vez."""
        if self._world_generado or self._generando:
            return

        # Verificar fix válido (STATUS_FIX = 0, STATUS_SBAS_FIX = 1, etc.)
        if msg.status.status < 0:
            self.get_logger().warn('GPS sin fix todavía, esperando...')
            return

        lat = msg.latitude
        lon = msg.longitude
        self.get_logger().info(f'✅ GPS recibido: lat={lat:.6f}, lon={lon:.6f}')
        self.get_logger().info('🗺️  Generando terreno (esto tarda ~30 seg)...')

        threading.Thread(
            target=self._generar_terreno,
            args=(lat, lon),
            daemon=True
        ).start()

    def _generar_terreno(self, lat: float, lon: float):
        """
        Genera el heightmap y el .world en un hilo separado
        para no bloquear el executor de ROS 2.
        """
        self._generando = True

        try:
            import numpy as np

            # Verificar dependencias
            try:
                import elevation
                import rasterio
                from PIL import Image
            except ImportError as e:
                self.get_logger().error(f'Dependencia faltante: {e}')
                self.get_logger().error('Ejecuta: pip install elevation rasterio Pillow')
                return

            dir_dem    = os.path.join(self.output_dir, 'data', 'dem')
            dir_worlds = os.path.join(self.output_dir, 'worlds')
            os.makedirs(dir_dem,    exist_ok=True)
            os.makedirs(dir_worlds, exist_ok=True)

            # 1) Descargar DEM
            delta_lat = self.radio_km / 111.0
            delta_lon = self.radio_km / (111.0 * np.cos(np.radians(lat)))

            bounds = (
                lon - delta_lon,
                lat - delta_lat,
                lon + delta_lon,
                lat + delta_lat
            )

            ruta_tif = os.path.join(dir_dem, 'terreno_auto.tif')
            self.get_logger().info(f'📡 Descargando SRTM... bounds={bounds}')
            elevation.clip(bounds=bounds, output=os.path.abspath(ruta_tif), product='SRTM3')
            elevation.clean()

            # 2) Procesar heightmap
            with rasterio.open(ruta_tif) as ds:
                datos = ds.read(1).astype(np.float32)
                t     = ds.transform
                res_x = abs(t.a) * 111320 * np.cos(np.radians(lat))
                res_y = abs(t.e) * 111320
                tam   = max(ds.width * res_x, ds.height * res_y)

            datos[datos < -1000] = np.nanmin(datos[datos > -1000])
            emin = float(np.nanmin(datos))
            emax = float(np.nanmax(datos))
            erng = emax - emin

            from PIL import Image as PILImage
            img = PILImage.fromarray(datos).resize(
                (self.resolucion, self.resolucion), PILImage.BILINEAR)
            datos_r = np.array(img)
            datos_n = ((datos_r - emin) / (erng if erng > 0 else 1) * 65535).astype(np.uint16)

            ruta_png = os.path.join(dir_dem, 'heightmap_auto.png')
            PILImage.fromarray(datos_n, mode='I;16').save(ruta_png)
            ruta_png_abs = os.path.abspath(ruta_png)

            # 3) Generar .world
            erng_z  = erng * self.escala_z
            z_off   = emin + erng_z / 2.0

            world = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="terreno_auto">
    <include><uri>model://sun</uri></include>
    <gravity>0 0 -9.81</gravity>
    <physics name="default_physics" default="0" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.53 0.81 0.98 1</background>
      <sky><clouds><speed>12</speed></clouds></sky>
    </scene>
    <model name="terreno">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <heightmap>
              <uri>file://{ruta_png_abs}</uri>
              <size>{tam:.2f} {tam:.2f} {erng_z:.2f}</size>
              <pos>0 0 {z_off:.2f}</pos>
            </heightmap>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <heightmap>
              <uri>file://{ruta_png_abs}</uri>
              <size>{tam:.2f} {tam:.2f} {erng_z:.2f}</size>
              <pos>0 0 {z_off:.2f}</pos>
              <texture>
                <diffuse>file://media/materials/textures/dirt_diffusespecular.png</diffuse>
                <normal>file://media/materials/textures/flat_normal.png</normal>
                <size>4</size>
              </texture>
            </heightmap>
          </geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>"""

            ruta_world = os.path.join(dir_worlds, 'terreno_auto.world')
            with open(ruta_world, 'w') as f:
                f.write(world)

            self.get_logger().info(f'✅ World generado: {ruta_world}')

            # 4) Publicar path
            msg_out = String()
            msg_out.data = os.path.abspath(ruta_world)
            self.pub_world.publish(msg_out)

            self._world_generado = True

        except Exception as e:
            self.get_logger().error(f'❌ Error generando terreno: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())

        finally:
            self._generando = False


def main(args=None):
    rclpy.init(args=args)
    node = GeneradorTerrenoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()