#!/bin/bash
# instalar_dependencias.sh
# Instala TODO lo necesario para el proyecto Gemelo Digital QAV250
# Plataforma destino: Ubuntu 22.04 LTS (Jammy Jellyfish)
# ROS Distro: ROS 2 Humble Hawksbill
# Simulador: Gazebo Classic 11
# Ejecutar con: bash instalar_dependencias.sh
#
# NOTA: El script detectará componentes ya instalados para evitar reinstalaciones redundantes.

set -e

# Colores para salida en consola
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # Sin color

echo -e "${YELLOW}══════════════════════════════════════════════════════════════${NC}"
echo -e "${GREEN}   INSTALADOR DE DEPENDENCIAS GLOBAL - GEMELO DIGITAL QAV250${NC}"
echo -e "   Ubuntu 22.04 LTS | ROS 2 Humble | Gazebo Classic 11"
echo -e "${YELLOW}══════════════════════════════════════════════════════════════${NC}"

# ── 1. CONFIGURACIÓN DE LOCALE (Requerido por ROS 2) ───────────────────
echo -e "\n${YELLOW}[1/6] Configurando Locale del Sistema...${NC}"
sudo apt-get update -qq
sudo apt-get install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
echo -e "${GREEN}[OK] Locale configurado correctamente.${NC}"

# ── 2. INSTALACIÓN DE ROS 2 HUMBLE ─────────────────────────────────────
echo -e "\n${YELLOW}[2/6] Verificando instalación de ROS 2 Humble...${NC}"
if [ -f /opt/ros/humble/setup.bash ]; then
    echo -e "${GREEN}[OK] ROS 2 Humble ya está instalado en /opt/ros/humble.${NC}"
else
    echo -e "${YELLOW}ROS 2 Humble no detectado. Iniciando instalación completa...${NC}"
    
    # Habilitar repositorio Ubuntu Universe
    sudo apt-get install -y software-properties-common
    sudo add-apt-repository -y universe
    
    # Agregar llave GPG de ROS 2
    sudo apt-get update -qq
    sudo apt-get install -y curl gnupg lsb-release
    sudo curl -sSL https://raw.githubusercontent.com/ros2/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    
    # Agregar repositorio a lista de fuentes
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
    # Instalar ROS 2 Humble Desktop (incluye herramientas visuales y rviz)
    echo -e "${YELLOW}Actualizando índices e instalando ROS 2 Humble Desktop (esto puede tardar)...${NC}"
    sudo apt-get update -qq
    sudo apt-get install -y ros-humble-desktop python3-colcon-common-extensions
    echo -e "${GREEN}[OK] ROS 2 Humble instalado correctamente.${NC}"
fi

# Hacer source temporal
source /opt/ros/humble/setup.bash

# ── 3. INSTALACIÓN DE GAZEBO CLASSIC 11 ────────────────────────────────
echo -e "\n${YELLOW}[3/6] Verificando instalación de Gazebo Classic 11...${NC}"
if command -v gzserver &> /dev/null; then
    echo -e "${GREEN}[OK] Gazebo Classic ya está instalado: $(gazebo --version | head -n 1).${NC}"
else
    echo -e "${YELLOW}Gazebo no detectado. Instalando Gazebo Classic 11...${NC}"
    # Agregar repositorio oficial de OSRF/Gazebo
    sudo curl -sSL http://packages.osrfoundation.org/gazebo.key | sudo apt-key add -
    echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list
    
    sudo apt-get update -qq
    sudo apt-get install -y gazebo11 libgazebo11-dev
    echo -e "${GREEN}[OK] Gazebo Classic 11 instalado correctamente.${NC}"
fi

# ── 4. INSTALACIÓN DE PAQUETES ROS 2 / SIMULACIÓN (APT) ────────────────
echo -e "\n${YELLOW}[4/6] Instalando integración Gazebo-ROS 2 y dependencias...${NC}"
sudo apt-get install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-msgs \
    ros-humble-tf-transformations \
    ros-humble-launch-ros \
    python3-pip \
    python3-numpy \
    python3-transforms3d
echo -e "${GREEN}[OK] Paquetes ROS 2 instalados.${NC}"

# ── 5. INSTALACIÓN DE LIBRERÍAS PYTHON (PIP) ───────────────────────────
echo -e "\n${YELLOW}[5/6] Instalando dependencias de Python (pip)...${NC}"
pip3 install --upgrade pip
pip3 install pymavlink scipy matplotlib
echo -e "${GREEN}[OK] Librerías Python instaladas.${NC}"

# ── 6. CONFIGURACIÓN DEL ENTORNO BASHRC ────────────────────────────────
echo -e "\n${YELLOW}[6/6] Configurando variables de entorno en ~/.bashrc...${NC}"
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    echo -e "${GREEN}[OK] ROS 2 Humble agregado a tu ~/.bashrc.${NC}"
fi

if [ -d ~/ros2_ws ]; then
    if ! grep -q "source ~/ros2_ws/install/setup.bash" ~/.bashrc; then
        echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
        echo -e "${GREEN}[OK] Workspace local agregado a tu ~/.bashrc.${NC}"
    fi
fi

# ── VERIFICACIÓN FINAL ─────────────────────────────────────────────────
echo -e "\n${YELLOW}══════════════════════════════════════════════════════════════${NC}"
echo -e "${GREEN}  ¡TODAS LAS DEPENDENCIAS HAN SIDO INSTALADAS Y CONFIGURADAS!${NC}"
echo -e "${YELLOW}══════════════════════════════════════════════════════════════${NC}"
echo -e "  Versiones del sistema detectadas:"
python3 -c "import numpy;          print(f'   - NumPy:         {numpy.__version__}')"
python3 -c "import pymavlink;      print(f'   - PyMavlink:     {pymavlink.__version__}')"
python3 -c "import transforms3d;   print(f'   - Transforms3D:  {transforms3d.__version__}')"
echo -e "   - Gazebo:        $(gzserver --version | head -n 1)"
echo -e "   - ROS 2 Distro:  $ROS_DISTRO"
echo -e "--------------------------------------------------------------"
echo -e "  Para comenzar a trabajar:"
echo -e "    1. Abre una nueva terminal o ejecuta: ${YELLOW}source ~/.bashrc${NC}"
echo -e "    2. Compila el proyecto:"
echo -e "       ${YELLOW}cd ~/ros2_ws && colcon build --packages-select gemelo_digital_qav250 --symlink-install${NC}"
echo -e "    3. Corre el simulador:"
echo -e "       - Modo DEMO (sin dron real): ${YELLOW}ros2 launch gemelo_digital_qav250 gemelo_demo.launch.py${NC}"
echo -e "       - Modo REAL (con Pixhawk):   ${YELLOW}ros2 launch gemelo_digital_qav250 gemelo_digital.launch.py${NC}"
echo -e "${YELLOW}══════════════════════════════════════════════════════════════${NC}"