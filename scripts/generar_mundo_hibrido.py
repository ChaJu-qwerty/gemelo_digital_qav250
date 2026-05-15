#!/usr/bin/env python3
"""
generar_mundo_hibrido.py  — v3
--------------------------------
EJECUTAR desde la raíz del paquete:
    cd ~/ros2_ws/src/gemelo_digital_qav250
    python3 scripts/generar_mundo_hibrido.py
"""

# ═══════════════════════════════════════════════════
#  CONFIGURACIÓN DEL USUARIO
# ═══════════════════════════════════════════════════
LAT_CENTRO       = 29.0729
LON_CENTRO       = -110.9559
RADIO_KM         = 2.0
RESOLUCION       = 129
ESCALA_Z         = 1.0
ALTURA_DEFAULT_M = 6.0
# ═══════════════════════════════════════════════════

import os, sys, math
import numpy as np

RAIZ       = os.getcwd()
DIR_DEM    = os.path.join(RAIZ, "data", "dem")
DIR_WORLDS = os.path.join(RAIZ, "worlds")

def bbox():
    dlat = RADIO_KM / 111.0
    dlon = RADIO_KM / (111.0 * math.cos(math.radians(LAT_CENTRO)))
    return (LON_CENTRO-dlon, LAT_CENTRO-dlat, LON_CENTRO+dlon, LAT_CENTRO+dlat)

def verificar_dependencias():
    faltantes = []
    for lib, pip in [("elevation","elevation"),("rasterio","rasterio"),
                     ("PIL","Pillow"),("osmnx","osmnx"),("shapely","shapely")]:
        try: __import__("PIL" if lib=="PIL" else lib)
        except ImportError: faltantes.append(pip)
    if faltantes:
        print(f"Faltan: pip install {' '.join(faltantes)}"); sys.exit(1)
    print("OK dependencias")

def crear_directorios():
    os.makedirs(DIR_DEM, exist_ok=True)
    os.makedirs(DIR_WORLDS, exist_ok=True)
    if "gemelo_digital_qav250" not in RAIZ:
        print("ADVERTENCIA: ejecuta desde ~/ros2_ws/src/gemelo_digital_qav250")
        if input("Continuar? (s/n): ").strip().lower() != 's': sys.exit(0)

# ── PASO 1: SRTM ───────────────────────────────────

def descargar_dem():
    import elevation
    bounds = bbox()
    ruta = os.path.join(DIR_DEM, "terreno.tif")
    print(f"\n[1/4] Descargando SRTM...")
    elevation.clip(bounds=bounds, output=os.path.abspath(ruta), product='SRTM3')
    elevation.clean()
    print(f"      OK -> {ruta}")
    return ruta

def dem_a_heightmap(ruta_tif):
    import rasterio
    from PIL import Image
    print(f"\n[2/4] Procesando heightmap...")
    with rasterio.open(ruta_tif) as ds:
        datos = ds.read(1).astype(np.float32)
        t     = ds.transform
        res_x = abs(t.a) * 111320 * math.cos(math.radians(LAT_CENTRO))
        res_y = abs(t.e) * 111320
        tam   = max(ds.width * res_x, ds.height * res_y)

    datos[datos < -500] = np.nanmin(datos[datos > -500])
    emin = float(np.nanmin(datos))
    emax = float(np.nanmax(datos))
    erng = emax - emin

    print(f"      elev {emin:.1f} m – {emax:.1f} m  (rango {erng:.1f} m)")
    print(f"      mundo {tam:.0f} x {tam:.0f} m")

    img_r   = Image.fromarray(datos).resize((RESOLUCION, RESOLUCION), Image.BILINEAR)
    datos_r = np.array(img_r, dtype=np.float32)
    datos_n = ((datos_r - emin) / (erng if erng > 0 else 1) * 255).astype(np.uint8)

    # Calcular Z promedio del centro (para colocar edificios)
    cy, cx = RESOLUCION // 2, RESOLUCION // 2
    ventana = datos_n[cy-5:cy+5, cx-5:cx+5]
    pixel_centro = float(ventana.mean())
    z_centro_gazebo = (pixel_centro / 255.0) * erng * ESCALA_Z
    print(f"      pixel centro: {pixel_centro:.1f}  →  Z terreno centro: {z_centro_gazebo:.1f} m")

    ruta_png = os.path.join(DIR_DEM, "heightmap.png")
    Image.fromarray(datos_n, mode='L').save(ruta_png)
    print(f"      OK -> {ruta_png}")
    return ruta_png, emin, emax, tam, z_centro_gazebo

# ── PASO 2: OSM ────────────────────────────────────

def descargar_edificios():
    import osmnx as ox
    print(f"\n[3/4] Descargando edificios OSM...")
    try:
        gdf = ox.features_from_point(
            (LAT_CENTRO, LON_CENTRO),
            tags={"building": True},
            dist=RADIO_KM * 1000
        )
    except Exception as e:
        print(f"      AVISO: {e}"); return None
    gdf = gdf[gdf.geometry.geom_type.isin(["Polygon","MultiPolygon"])]
    print(f"      {len(gdf)} edificios encontrados")
    return gdf

def safe_altura(row):
    """Extrae altura válida del edificio, nunca devuelve nan."""
    for campo in ["height", "building:height", "building:levels"]:
        if campo in row.index and row[campo] is not None:
            try:
                v = str(row[campo]).strip().replace("m","").replace(",",".")
                f = float(v)
                if not math.isfinite(f) or f <= 0:
                    continue
                if campo == "building:levels":
                    f = f * 3.0
                return min(max(f, 2.0), 150.0)  # entre 2m y 150m
            except:
                pass
    return ALTURA_DEFAULT_M

def gdf_a_sdf(gdf, erng, z_centro_gazebo):
    if gdf is None or len(gdf) == 0:
        return "", 0

    bloques  = []
    saltados = 0

    for idx, (_, row) in enumerate(gdf.iterrows()):
        geom   = row.geometry
        altura = safe_altura(row)

        polys = list(geom.geoms) if geom.geom_type == "MultiPolygon" else [geom]

        for pidx, poly in enumerate(polys):
            coords_geo = list(poly.exterior.coords)

            coords_m = []
            for lon, lat in coords_geo:
                x = (lon - LON_CENTRO) * 111320 * math.cos(math.radians(LAT_CENTRO))
                y = (lat - LAT_CENTRO) * 111320
                coords_m.append((x, y))

            if len(coords_m) < 3:
                saltados += 1; continue

            cx_ed = sum(c[0] for c in coords_m) / len(coords_m)
            cy_ed = sum(c[1] for c in coords_m) / len(coords_m)
            xs    = [c[0] for c in coords_m]
            ys    = [c[1] for c in coords_m]
            ancho = max(xs) - min(xs)
            largo = max(ys) - min(ys)

            radio_m = RADIO_KM * 1000
            if abs(cx_ed) > radio_m or abs(cy_ed) > radio_m:
                saltados += 1; continue
            if ancho < 1.0 or largo < 1.0:
                saltados += 1; continue

            # Verificar que todas las dimensiones sean finitas
            vals = [cx_ed, cy_ed, ancho, largo, altura]
            if not all(math.isfinite(v) for v in vals):
                saltados += 1; continue

            # ── Z correcto ────────────────────────────────────────
            # El terreno en Gazebo va de Z=0 (emin) a Z=erng (emax).
            # Usamos z_centro_gazebo (calculado del pixel central del
            # heightmap) como la elevación base del área urbana.
            # El centro del edificio va a esa base + mitad de su altura.
            # ─────────────────────────────────────────────────────
            z_base   = z_centro_gazebo
            z_edificio = z_base + altura / 2.0

            nombre = f"ed_{idx}_{pidx}"
            bloque = f"""
    <model name="{nombre}">
      <static>true</static>
      <pose>{cx_ed:.2f} {cy_ed:.2f} {z_edificio:.2f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>{max(ancho,2):.2f} {max(largo,2):.2f} {altura:.2f}</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>{max(ancho,2):.2f} {max(largo,2):.2f} {altura:.2f}</size></box>
          </geometry>
          <material>
            <script>
              <uri>file:///usr/share/gazebo-11/media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
      </link>
    </model>"""
            bloques.append(bloque)

    print(f"      {len(bloques)} edificios OK  |  {saltados} saltados")
    return "\n".join(bloques), len(bloques)

# ── PASO 3: world ──────────────────────────────────

def generar_world(ruta_png, emin, emax, tam, sdf_edificios, n_ed, z_centro_gazebo):
    erng         = (emax - emin) * ESCALA_Z
    ruta_png_abs = os.path.abspath(ruta_png)
    print(f"\n[4/4] Generando .world  ({n_ed} edificios, Z_base={z_centro_gazebo:.1f} m)")

    # Cámara: 300m al sur del centro, 200m de altura, mirando al norte
    cam_z = z_centro_gazebo + 200

    world = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="hermosillo_hibrido">

    <include><uri>model://sun</uri></include>
    <gravity>0 0 -9.81</gravity>

    <physics name="default_physics" default="0" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <scene>
      <ambient>0.8 0.8 0.8 1</ambient>
      <background>0.53 0.81 0.98 1</background>
      <sky><clouds><speed>8</speed></clouds></sky>
      <shadows>true</shadows>
    </scene>

    <!--
      SRTM + OSM — Hermosillo, Sonora
      Centro    : {LAT_CENTRO} N  {LON_CENTRO} E
      Radio     : {RADIO_KM} km
      Elev real : {emin:.0f} – {emax:.0f} m  (rango {erng:.0f} m)
      Z terreno centro (Gazebo): {z_centro_gazebo:.1f} m
      Edificios : {n_ed}
    -->

    <!-- TERRENO SRTM -->
    <model name="terreno_hermosillo">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <heightmap>
              <uri>file://{ruta_png_abs}</uri>
              <size>{tam:.2f} {tam:.2f} {erng:.2f}</size>
              <pos>0 0 0</pos>
            </heightmap>
          </geometry>
        </collision>
        <visual name="visual">
          <material>
            <script>
              <uri>file:///usr/share/gazebo-11/media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Green</name>
            </script>
          </material>
          <geometry>
            <heightmap>
              <use_terrain_paging>false</use_terrain_paging>
              <uri>file://{ruta_png_abs}</uri>
              <size>{tam:.2f} {tam:.2f} {erng:.2f}</size>
              <pos>0 0 0</pos>
            </heightmap>
          </geometry>
        </visual>
      </link>
    </model>

    <!-- EDIFICIOS OSM -->
{sdf_edificios}

    <!-- CAMARA sobre el centro urbano -->
    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>0 -600 {cam_z:.1f} 0 0.6 1.57</pose>
        <view_controller>orbit</view_controller>
      </camera>
    </gui>

  </world>
</sdf>
"""
    ruta_world = os.path.join(DIR_WORLDS, "hermosillo_hibrido.world")
    with open(ruta_world, "w") as f:
        f.write(world)
    print(f"      OK -> {ruta_world}")
    return ruta_world

# ── MAIN ───────────────────────────────────────────

def main():
    print("=" * 60)
    print("  Generador Híbrido SRTM + OSM — Gazebo  v3")
    print(f"  Centro : ({LAT_CENTRO}, {LON_CENTRO})")
    print(f"  Radio  : {RADIO_KM} km")
    print("=" * 60)
    verificar_dependencias()
    crear_directorios()
    ruta_tif                       = descargar_dem()
    ruta_png, emin, emax, tam, z_c = dem_a_heightmap(ruta_tif)
    gdf                            = descargar_edificios()
    erng                           = (emax - emin) * ESCALA_Z
    sdf_edificios, n_ed            = gdf_a_sdf(gdf, erng, z_c)
    ruta_world                     = generar_world(ruta_png, emin, emax, tam, sdf_edificios, n_ed, z_c)
    print()
    print("=" * 60)
    print("  LISTO. Abre en Gazebo:")
    print(f"  killall gzserver gzclient 2>/dev/null; sleep 2")
    print(f"  gzserver {ruta_world} &")
    print(f"  sleep 8 && gzclient")
    print("=" * 60)

if __name__ == "__main__":
    main()