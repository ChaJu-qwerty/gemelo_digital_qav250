#!/bin/bash
# instalar_dependencias.sh
# Instala TODAS las dependencias necesarias para el gemelo digital QAV250
# Plataforma destino: Ubuntu 22.04 + ROS 2 Humble + Gazebo Classic 11
# Ejecutar con: bash instalar_dependencias.sh

set -e

echo "══════════════════════════════════════════════════════════════"
echo "  Instalando dependencias del Gemelo Digital QAV250"
echo "  Ubuntu 22.04 | ROS 2 Humble | Gazebo Classic 11"
echo "══════════════════════════════════════════════════════════════"

# ── 1. Verificar que ROS 2 Humble esté sourced ─────────────────
if [ -z "$ROS_DISTRO" ]; then
    echo "[WARN] ROS 2 no detectado. Haciendo source de Humble..."
    source /opt/ros/humble/setup.bash
fi
echo "[OK] ROS 2 distribución: $ROS_DISTRO"

# ── 2. Paquetes del sistema (apt) ───────────────────────────────
echo ""
echo "Instalando paquetes de sistema..."
sudo apt-get update -qq
sudo apt-get install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-msgs \
    ros-humble-tf-transformations \
    ros-humble-launch-ros \
    python3-pip \
    python3-numpy \
    python3-transforms3d

# ── 3. Dependencias Python (pip) ────────────────────────────────
echo ""
echo "Instalando librerías Python..."
pip3 install pymavlink

# ── 4. Verificar versiones instaladas ──────────────────────────
echo ""
echo "══════════════════════════════════════════════════════════════"
echo "  Verificando versiones..."
echo "══════════════════════════════════════════════════════════════"
python3 -c "import numpy as np;       print(f'  numpy          : {np.__version__}')"
python3 -c "import pymavlink;         print(f'  pymavlink      : {pymavlink.__version__}')"
python3 -c "import transforms3d;      print(f'  transforms3d   : {transforms3d.__version__}')"
python3 -c "from tf_transformations import quaternion_from_euler; print('  tf_transformations : OK')"

echo ""
echo "══════════════════════════════════════════════════════════════"
echo "  ¡Listo! Todas las dependencias instaladas."
echo ""
echo "  Para compilar el paquete ROS 2, ejecuta:"
echo "    cd ~/ros2_ws"
echo "    colcon build --packages-select gemelo_digital_qav250"
echo "    source install/setup.bash"
echo ""
echo "  Para lanzar la simulación:"
echo "    ros2 launch gemelo_digital_qav250 gemelo_digital.launch.py"
echo "══════════════════════════════════════════════════════════════"