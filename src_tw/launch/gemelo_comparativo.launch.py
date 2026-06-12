import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_gemelo_twin = get_package_share_directory('gemelo_digital_qav250')

    models_path = os.path.join(pkg_gemelo_twin, 'models')
    world_path = os.path.join(pkg_gemelo_twin, 'worlds', 'qav250_twin.world')

    kill_gazebo = ExecuteProcess(
        cmd=['bash', '-c', 'pkill -9 -f gzserver; pkill -9 -f gzclient; sleep 1'],
        output='screen'
    )

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

    # Dron principal (IMU Pixhawk)
    spawn_drone_1 = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-entity', 'drone_demo', 
                           '-file', os.path.join(models_path, 'drone_demo', 'drone_simple.sdf'),
                           '-x', '0.0', '-y', '0.0', '-z', '0.05',
                           '-timeout', '60'],
                output='screen'
            )
        ]
    )



    params_file = os.path.join(pkg_gemelo_twin, 'config', 'qav250_params.yaml')

    # El nodo principal se conecta al Pixhawk
    nodo_principal = Node(
        package='gemelo_digital_qav250',
        executable='nodo_gemelo_digital',
        parameters=[params_file, {"topic_pose": "/qav250/pose"}],
        output='screen'
    )

    # El lector del Gyro FFT
    nodo_gyro = Node(
        package='gemelo_digital_qav250',
        executable='nodo_lector_gyro',
        parameters=[{"puerto_serial": "/dev/ttyACM0"}],
        output='screen'
    )

    # El nodo fantasma usa el PWM del principal, pero la IMU del gyro
    nodo_fantasma = Node(
        package='gemelo_digital_qav250',
        executable='nodo_gemelo_fantasma',
        parameters=[params_file, {"topic_pose": "/qav250_fantasma/pose"}],
        output='screen'
    )

    return LaunchDescription([
        kill_gazebo,
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=[models_path, ':', os.environ.get('GAZEBO_MODEL_PATH', '')]
        ),
        gazebo,
        spawn_drone_1,
        # Si no tienes un SDF distinto, usa el mismo, Gazebo cambiará de color en Rviz pero en Gazebo se verán igual
        # He puesto 'drone_simple_ghost.sdf' por si lo quieres duplicar y cambiarle de color, si no existe fallará.
        # Mejor usar el mismo para evitar errores:
        TimerAction(
            period=15.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=['-entity', 'drone_fantasma', 
                               '-file', os.path.join(models_path, 'drone_demo', 'drone_simple_ghost.sdf'),
                               '-x', '0.0', '-y', '0.0', '-z', '0.05',
                               '-timeout', '60'],
                    output='screen'
                )
            ]
        ),
        nodo_principal,
        nodo_gyro,
        nodo_fantasma
    ])
