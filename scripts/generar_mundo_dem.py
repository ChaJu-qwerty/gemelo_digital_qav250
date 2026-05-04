#!/usr/bin/env python3
"""
generar_mundo_dem.py  — versión corregida
-----------------------------------------
Ejecutar SIEMPRE desde la raíz del paquete:
    cd ~/ros2_ws/src/gemelo_digital_qav250
    python3 scripts/generar_mundo_dem.py

Los archivos se generan en:
    data/dem/heightmap.png
    worlds/terreno_hermosillo.world
"""

import os
import sys
import numpy as np

# ─────────────────────────────────────────────
#  CONFIGURACIÓN
# ─────────────────────────────────────────────
LAT_CENTRO = 29.0729
LON_CENTRO = -110.9559
RADIO_KM   = 3.0
RESOLUCION = 129      # debe ser 2^n+1: 65, 129, 257, 513
ESCALA_Z   = 1.0
# ─────────────────────────────────────────────

# Raíz del paquete: siempre relativo a donde se LLAMA el script
RAIZ    = os.getcwd()
DIR_DEM    = os.path.join(RAIZ, "data", "dem")
DIR_WORLDS = os.path.join(RAIZ, "worlds")


def verificar_dependencias():
    faltantes = []
    for lib in ["requests", "elevation", "PIL", "rasterio"]:
        try:
            __import__("PIL" if lib == "PIL" else lib)
        except ImportError:
            faltantes.append("Pillow" if lib == "PIL" else lib)
    if faltantes:
        print("Faltan dependencias. Instalalas con:")
        print(f"   pip install {' '.join(faltantes)}")
        sys.exit(1)
    print("OK Dependencias OK")


def crear_directorios():
    os.makedirs(DIR_DEM,    exist_ok=True)
    os.makedirs(DIR_WORLDS, exist_ok=True)
    print(f"OK Directorios creados")
    print(f"   DEM    -> {DIR_DEM}")
    print(f"   Worlds -> {DIR_WORLDS}")


def descargar_dem():
    import elevation

    delta_lat = RADIO_KM / 111.0
    delta_lon = RADIO_KM / (111.0 * np.cos(np.radians(LAT_CENTRO)))

    bounds = (
        LON_CENTRO - delta_lon,
        LAT_CENTRO - delta_lat,
        LON_CENTRO + delta_lon,
        LAT_CENTRO + delta_lat,
    )

    ruta_tif = os.path.join(DIR_DEM, "terreno_hermosillo.tif")
    print(f"\nDescargando SRTM...")
    print(f"   bounds : {tuple(round(b,5) for b in bounds)}")
    print(f"   salida : {ruta_tif}")

    elevation.clip(bounds=bounds, output=os.path.abspath(ruta_tif), product='SRTM3')
    elevation.clean()

    print(f"OK DEM descargado")
    return ruta_tif


def dem_a_heightmap(ruta_tif):
    import rasterio
    from PIL import Image

    print("\nProcesando elevacion...")

    with rasterio.open(ruta_tif) as ds:
        datos = ds.read(1).astype(np.float32)
        t     = ds.transform
        res_x = abs(t.a) * 111320 * np.cos(np.radians(LAT_CENTRO))
        res_y = abs(t.e) * 111320
        tam   = max(ds.width * res_x, ds.height * res_y)

    # Limpiar valores invalidos (nodata SRTM = -32768)
    datos[datos < -500] = np.nanmin(datos[datos > -500])

    emin = float(np.nanmin(datos))
    emax = float(np.nanmax(datos))
    erng = emax - emin

    print(f"   Elevacion min : {emin:.1f} m")
    print(f"   Elevacion max : {emax:.1f} m")
    print(f"   Rango         : {erng:.1f} m")
    print(f"   Tamano mundo  : {tam:.0f} x {tam:.0f} m")

    img_r   = Image.fromarray(datos).resize((RESOLUCION, RESOLUCION), Image.BILINEAR)
    datos_r = np.array(img_r)
    datos_n = ((datos_r - emin) / (erng if erng > 0 else 1) * 65535).astype(np.uint16)

    ruta_png = os.path.join(DIR_DEM, "heightmap.png")
    Image.fromarray(datos_n, mode='I;16').save(ruta_png)

    print(f"OK Heightmap guardado: {ruta_png}")
    return ruta_png, emin, emax, tam


def generar_world(ruta_png, emin, emax, tam):
    erng         = (emax - emin) * ESCALA_Z
    z_off        = emin + erng / 2.0
    ruta_png_abs = os.path.abspath(ruta_png)

    world = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="terreno_hermosillo">

    <include>
      <uri>model://sun</uri>
    </include>

    <gravity>0 0 -9.81</gravity>
    <physics name="default_physics" default="0" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.53 0.81 0.98 1</background>
      <sky><clouds><speed>8</speed></clouds></sky>
      <shadows>true</shadows>
    </scene>

    <!--
      Terreno SRTM - Hermosillo, Sonora
      Centro : {LAT_CENTRO}N  {LON_CENTRO}E
      Area   : {RADIO_KM*2:.1f} x {RADIO_KM*2:.1f} km
      Elev   : {emin:.0f} m - {emax:.0f} m
    -->
    <model name="terreno_hermosillo">
      <static>true</static>
      <link name="link">

        <collision name="collision">
          <geometry>
            <heightmap>
              <uri>file://{ruta_png_abs}</uri>
              <size>{tam:.2f} {tam:.2f} {erng:.2f}</size>
              <pos>0 0 {z_off:.2f}</pos>
            </heightmap>
          </geometry>
        </collision>

        <!-- Visual SIN texturas externas para evitar el circulo azul -->
        <visual name="visual">
          <geometry>
            <heightmap>
              <use_terrain_paging>false</use_terrain_paging>
              <uri>file://{ruta_png_abs}</uri>
              <size>{tam:.2f} {tam:.2f} {erng:.2f}</size>
              <pos>0 0 {z_off:.2f}</pos>
            </heightmap>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>

      </link>
    </model>

  </world>
</sdf>
"""

    ruta_world = os.path.join(DIR_WORLDS, "terreno_hermosillo.world")
    with open(ruta_world, "w") as f:
        f.write(world)

    print(f"OK World generado: {ruta_world}")
    return ruta_world


def main():
    print("=" * 58)
    print("  Generador de Terreno SRTM - Hermosillo, Sonora")
    print(f"  Ejecutando desde: {RAIZ}")
    print("=" * 58)

    esperado = "gemelo_digital_qav250"
    if esperado not in RAIZ:
        print(f"\nADVERTENCIA: No estas en la raiz del paquete.")
        print(f"   Ejecuta asi:")
        print(f"   cd ~/ros2_ws/src/gemelo_digital_qav250")
        print(f"   python3 scripts/generar_mundo_dem.py\n")
        resp = input("Continuar de todas formas? (s/n): ").strip().lower()
        if resp != 's':
            sys.exit(0)

    verificar_dependencias()
    crear_directorios()
    ruta_tif             = descargar_dem()
    ruta_png, emin, emax, tam = dem_a_heightmap(ruta_tif)
    ruta_world           = generar_world(ruta_png, emin, emax, tam)

    print()
    print("=" * 58)
    print("  Listo! Para ver en Gazebo:")
    print()
    print(f"  gzserver {ruta_world} &")
    print(f"  gzclient")
    print("=" * 58)


if __name__ == "__main__":
    main()