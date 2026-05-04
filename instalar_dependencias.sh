#!/bin/bash
# instalar_dependencias.sh
# Instala todas las dependencias necesarias para el generador de terreno
# Ejecutar con: bash instalar_dependencias.sh

set -e
echo "══════════════════════════════════════════════"
echo "  Instalando dependencias para terreno SRTM"
echo "══════════════════════════════════════════════"

# ── Python libs ────────────────────────────────────
echo ""
echo "Instalando librerías Python..."
pip install elevation rasterio Pillow numpy requests

# ── GDAL (necesario para rasterio y elevation) ─────
echo ""
echo "Instalando GDAL del sistema..."
sudo apt-get update -qq
sudo apt-get install -y gdal-bin python3-gdal libgdal-dev

# Verificar versiones
echo ""
echo "Versiones instaladas:"
python3 -c "import elevation; print(f'  elevation : OK')"
python3 -c "import rasterio;  print(f'  rasterio  : {rasterio.__version__}')"
python3 -c "from PIL import Image; import PIL; print(f'  Pillow    : {PIL.__version__}')"
python3 -c "import numpy as np;    print(f'  numpy     : {np.__version__}')"

echo ""
echo "══════════════════════════════════════════════"
echo "   Todo instalado. Ahora se puede ejecutar:"
echo "     python3 generar_mundo_dem.py"
echo "══════════════════════════════════════════════"