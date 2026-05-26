# Gemelo Digital QAV250 — Modelación de Mundo 3D

Módulo de generación de entornos 3D realistas para Gazebo combinando datos de elevación satelital (NASA SRTM) y edificios de OpenStreetMap. Permite simular el entorno real donde opera el dron QAV250.

---

## ¿Qué hace este módulo?

Dado una coordenada GPS central y un radio en kilómetros, el sistema:

1. Descarga datos de elevación SRTM (NASA) para generar el terreno real
2. Descarga edificios de OpenStreetMap y los extruye en 3D
3. Genera un archivo `.world` listo para Gazebo con terreno + edificios

```
Coordenada GPS (fija en código o tomada del Pixhawk)
         ↓
Descarga tile SRTM de la zona  +  Edificios OSM
         ↓
Genera heightmap.png  +  modelos SDF de edificios
         ↓
Exporta hermosillo_hibrido.world
         ↓
Gazebo — dron QAV250 vuela sobre terreno real
```

---

## Estructura de archivos

```
gemelo_digital_qav250/
├── scripts/
│   ├── generar_mundo_hibrido.py     # Script principal — genera el .world
│   └── rutina_drone.py              # Rutina de vuelo de prueba
│
├── models/
│   └── drone_demo/
│       ├── model.config             # Metadata del modelo
│       └── drone_simple.sdf        # Modelo 3D del drone de prueba (rojo)
│
├── data/dem/                        # Generado automáticamente
│   ├── terreno.tif                  # Datos SRTM crudos
│   └── heightmap.png                # Imagen de altura para Gazebo
│
└── worlds/                          # Generado automáticamente
    └── hermosillo_hibrido.world     # Mundo final para Gazebo
```

---

## Dependencias

```bash
# Sistema
sudo apt-get install gdal-bin python3-gdal libgdal-dev

# Python
pip install elevation rasterio Pillow numpy osmnx shapely

# Si hay conflicto de NumPy
pip install "numpy<2.0" --force-reinstall
```

---

## Uso

### 1. Configurar coordenadas y radio

Edita las primeras líneas de `scripts/generar_mundo_hibrido.py`:

```python
LAT_CENTRO       = 29.0729    # Latitud central
LON_CENTRO       = -110.9559  # Longitud central
RADIO_KM         = 2.0        # Radio del área en km (recomendado: 1–3)
RESOLUCION       = 129        # Resolución heightmap: 65 / 129 / 257 / 513
ESCALA_Z         = 1.0        # Multiplicador de altura (1.0 = real)
ALTURA_DEFAULT_M = 6.0        # Altura de edificios sin dato en OSM (m)
```

### 2. Generar el mundo

```bash
cd ~/ros2_ws/src/gemelo_digital_qav250
python3 scripts/generar_mundo_hibrido.py
```

Salida esperada:
```
[1/4] Descargando SRTM...       ← ~30 seg, requiere internet
[2/4] Procesando heightmap...
      elev 189 m – 418 m  (rango 229 m)
      mundo 3989 x 3989 m
[3/4] Descargando edificios OSM...
      433 edificios encontrados
[4/4] Generando .world  (433 edificios)
      OK -> worlds/hermosillo_hibrido.world
```

### 3. Abrir en Gazebo

```bash
export GAZEBO_MODEL_PATH=/home/$USER/ros2_ws/src/gemelo_digital_qav250/models:$GAZEBO_MODEL_PATH

killall gzserver gzclient 2>/dev/null; sleep 2
cd ~/ros2_ws/src/gemelo_digital_qav250
gzserver worlds/hermosillo_hibrido.world &
sleep 8 && gzclient
```

### 4. Correr rutina de vuelo de prueba

Con Gazebo ya abierto, en otra terminal:

```bash
cd ~/ros2_ws/src/gemelo_digital_qav250
source /opt/ros/humble/setup.bash
python3 scripts/rutina_drone.py
```

El drone ejecuta: despegue → cuadrado de 150×150 m → regreso → aterrizaje.

---

## Fuentes de datos

| Fuente | Uso | Resolución | Licencia |
|--------|-----|------------|----------|
| NASA SRTM3 | Elevación del terreno | ~90 m/pixel | Dominio público |
| OpenStreetMap | Edificios y footprints | Variable | ODbL |

> **Nota:** SRTM captura topografía natural (cerros, valles). Los edificios provienen de OSM — la cobertura depende de los contribuidores locales. En Hermosillo centro hay ~430 edificios mapeados.

---

## Parámetros clave

| Parámetro | Descripción | Valor actual |
|-----------|-------------|--------------|
| `LAT_CENTRO` | Latitud del centro del mapa | 29.0729 |
| `LON_CENTRO` | Longitud del centro del mapa | -110.9559 |
| `RADIO_KM` | Radio del área generada | 2.0 km |
| `RESOLUCION` | Resolución del heightmap | 129×129 px |
| `ALTURA_DEFAULT_M` | Altura de edificios sin dato | 6.0 m |
| `Z_BASE` (rutina) | Altura de vuelo en Gazebo | 50.0 m |

---

## Trabajo futuro

- [ ] Integración con GPS del Pixhawk via MAVROS para centrar el mapa automáticamente
- [ ] Texturas de satélite sobre el terreno (Mapbox / WMTS)
- [ ] Evitación de obstáculos usando el mapa de edificios OSM
- [ ] Mayor resolución con datos LiDAR de INEGI para zonas específicas
- [ ] Cámara de seguimiento adjunta al modelo del dron

---

## Notas técnicas

- El heightmap se genera en formato PNG de 8 bits escala de grises (modo `L`), que es el formato más compatible con Gazebo 11
- Los edificios OSM se aproximan como cajas (`<box>`) usando el bounding box de cada polígono
- El Z de los edificios se calcula a partir del pixel central del heightmap para coincidir con la elevación real del terreno en Gazebo
- El `GAZEBO_MODEL_PATH` debe incluir la carpeta `models/` del paquete para que Gazebo encuentre el modelo `drone_demo`
