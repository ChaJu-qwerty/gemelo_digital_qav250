"""
gazebo_terreno_hermosillo.launch.py
------------------------------------
Launch file que:
  1) Lanza el nodo generador_terreno_node (genera el .world)
  2) Espera a que el .world esté listo
  3) Lanza Gazebo con el terreno de Hermosillo

Uso:
    # Con coordenada fija (sin Pixhawk):
    ros2 launch gemelo_digital_qav250 gazebo_terreno_hermosillo.launch.py

    # Con GPS real del Pixhawk:
    ros2 launch gemelo_digital_qav250 gazebo_terreno_hermosillo.launch.py usar_gps:=true

    # Con coordenada personalizada:
    ros2 launch gemelo_digital_qav250 gazebo_terreno_hermosillo.launch.py \\
        lat_fija:=29.1000 lon_fija:=-110.9000 radio_km:=5.0
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory('gemelo_digital_qav250')

    # ── Argumentos del launch ──────────────────────────────────────
    arg_usar_gps  = DeclareLaunchArgument('usar_gps',   default_value='false',
                        description='true=usar GPS Pixhawk, false=coordenada fija')
    arg_lat       = DeclareLaunchArgument('lat_fija',   default_value='29.0729',
                        description='Latitud central del mapa')
    arg_lon       = DeclareLaunchArgument('lon_fija',   default_value='-110.9559',
                        description='Longitud central del mapa')
    arg_radio     = DeclareLaunchArgument('radio_km',   default_value='3.0',
                        description='Radio del área a generar (km)')
    arg_resol     = DeclareLaunchArgument('resolucion', default_value='129',
                        description='Resolución del heightmap (65/129/257/513)')
    arg_escala_z  = DeclareLaunchArgument('escala_z',   default_value='1.0',
                        description='Multiplicador de altura')
    arg_world     = DeclareLaunchArgument('world',
                        default_value=os.path.join(
                            os.path.expanduser('~'),
                            'ros2_ws', 'src', 'gemelo_digital_qav250',
                            'worlds', 'terreno_hermosillo.world'),
                        description='Path al .world (si ya lo generaste manualmente)')

    # ── Nodo generador de terreno ──────────────────────────────────
    nodo_generador = Node(
        package='gemelo_digital_qav250',
        executable='generador_terreno_node',
        name='generador_terreno_node',
        output='screen',
        parameters=[{
            'usar_gps':    LaunchConfiguration('usar_gps'),
            'lat_fija':    LaunchConfiguration('lat_fija'),
            'lon_fija':    LaunchConfiguration('lon_fija'),
            'radio_km':    LaunchConfiguration('radio_km'),
            'resolucion':  LaunchConfiguration('resolucion'),
            'escala_z':    LaunchConfiguration('escala_z'),
            'output_dir':  os.path.join(
                               os.path.expanduser('~'),
                               'ros2_ws', 'src', 'gemelo_digital_qav250'),
        }]
    )

    # ── Gazebo (se lanza después de 45 seg para dar tiempo al generador) ──
    # Si ya tienes el world pre-generado, cambia el TimerAction a 5 seg
    gazebo = TimerAction(
        period=45.0,
        actions=[
            ExecuteProcess(
                cmd=['gazebo', '--verbose',
                     LaunchConfiguration('world')],
                output='screen',
                additional_env={'GAZEBO_MODEL_PATH':
                    os.path.join(pkg_share, 'models') + ':' +
                    os.environ.get('GAZEBO_MODEL_PATH', '')}
            )
        ]
    )

    return LaunchDescription([
        arg_usar_gps,
        arg_lat,
        arg_lon,
        arg_radio,
        arg_resol,
        arg_escala_z,
        arg_world,
        nodo_generador,
        gazebo,
    ])