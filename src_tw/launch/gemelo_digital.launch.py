import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Obtener directorios de instalación de share
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_gemelo_twin = get_package_share_directory('gemelo_digital_qav250')

    # 2. Configurar la ruta de los modelos de Gazebo para que encuentre el drone_demo
    models_path = os.path.join(pkg_gemelo_twin, 'models')
    
    # 3. Ruta del archivo de mundo
    world_path = os.path.join(pkg_gemelo_twin, 'worlds', 'qav250_twin.world')
    
    # 4. Lanzar Gazebo con el mundo configurado
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    # 5. Nodo para spawnear el drone en Gazebo
    spawn_drone = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'drone_demo', 
                   '-file', os.path.join(models_path, 'drone_demo', 'drone_simple.sdf'),
                   '-x', '0.0', '-y', '0.0', '-z', '0.0',
                   '-timeout', '120'],
        output='screen'
    )

    # 6. Ruta del archivo de parámetros del nodo gemelo digital
    params_file = os.path.join(pkg_gemelo_twin, 'config', 'qav250_params.yaml')

    # 7. Lanzar el nodo del gemelo digital
    nodo_gemelo = Node(
        package='gemelo_digital_qav250',
        executable='nodo_gemelo_digital',
        parameters=[params_file],
        output='screen'
    )

    return LaunchDescription([
        # Modificar variable de entorno GAZEBO_MODEL_PATH para que Gazebo encuentre el modelo
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=[models_path, ':', os.environ.get('GAZEBO_MODEL_PATH', '')]
        ),
        gazebo,
        spawn_drone,
        nodo_gemelo
    ])
