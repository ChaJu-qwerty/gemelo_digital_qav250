#!/bin/bash
# instalar_dependencias.sh
# Instala todas las dependencias necesarias para el generador de terreno
# Ejecutar con: bash instalar_dependencias.sh

set -e
echo "══════════════════════════════════════════════"
echo "  Instalando dependencias para gemelo digital"
echo "══════════════════════════════════════════════"

# ── Python libs ────────────────────────────────────
echo ""
echo "Instalando librerías Python..."
pip install numpy pymavlink


# Verificar versiones
echo ""
echo "Versiones instaladas:"
python3 -c "import numpy as np;    print(f'  numpy     : {np.__version__}')"

echo ""
echo "══════════════════════════════════════════════"
echo "   Todo instalado. Ahora se puede ejecutar:"
echo "     python3 generar_mundo_dem.py"
echo "══════════════════════════════════════════════"