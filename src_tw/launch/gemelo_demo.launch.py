"""
Launch file para probar el gemelo digital SIN drone físico.

Lanza:
1. Gazebo Classic con el mundo qav250_twin.world (incluye objetos de referencia)
2. Spawn del modelo 3D del dron (con delay para esperar que Gazebo arranque)
3. Nodo gemelo digital en MODO DEMO (sin MAVLink)
4. Nodo rutina_demo (genera PWM simulados automáticamente)

Uso:
    ros2 launch gemelo_digital_qav250 gemelo_demo.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Obtener directorios de instalación de share
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_gemelo_twin = get_package_share_directory('gemelo_digital_qav250')

    # 2. Configurar la ruta de los modelos de Gazebo
    models_path = os.path.join(pkg_gemelo_twin, 'models')
    
    # 3. Ruta del archivo de mundo
    world_path = os.path.join(pkg_gemelo_twin, 'worlds', 'qav250_twin.world')
    sdf_path   = os.path.join(models_path, 'drone_demo', 'drone_simple.sdf')
    params_file = os.path.join(pkg_gemelo_twin, 'config', 'qav250_params.yaml')

    # [NUEVO] Matar procesos zombie de Gazebo que puedan bloquear el puerto
    kill_gazebo = ExecuteProcess(
        cmd=['bash', '-c', 'pkill -9 -f gzserver; pkill -9 -f gzclient; sleep 1'],
        output='screen'
    )

    # 4. Lanzar Gazebo con el mundo configurado (con delay para que el kill termine)
    gazebo = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
                ),
                launch_arguments={'world': world_path}.items()
            )
        ]
    )

    # 5. Spawn del drone con delay de 15s para que Gazebo termine de cargar en PCs lentas
    spawn_drone = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'drone_demo',
                    '-file', sdf_path,
                    '-x', '0.0', '-y', '0.0', '-z', '0.05',
                    '-timeout', '60',
                ],
                output='screen',
            )
        ]
    )

    # 6. Nodo gemelo digital EN MODO DEMO (con delay extra)
    nodo_gemelo = TimerAction(
        period=18.0,
        actions=[
            Node(
                package='gemelo_digital_qav250',
                executable='nodo_gemelo_digital',
                parameters=[
                    params_file,
                    {'modo_demo': True},  # ← Activar modo demo (sin MAVLink)
                ],
                output='screen',
            )
        ]
    )

    # 7. Rutina de demostración (empieza mucho después para asegurar que Gazebo se ve)
    nodo_demo = TimerAction(
        period=25.0,
        actions=[
            Node(
                package='gemelo_digital_qav250',
                executable='rutina_demo',
                parameters=[params_file],
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        # Matar procesos viejos de Gazebo antes de iniciar
        kill_gazebo,
        # Configurar GAZEBO_MODEL_PATH para encontrar el modelo del dron
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=[models_path, ':', os.environ.get('GAZEBO_MODEL_PATH', '')]
        ),
        gazebo,
        spawn_drone,
        nodo_gemelo,
        nodo_demo,
    ])
