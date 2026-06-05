import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'gemelo_digital_qav250'

# Función para encontrar recursivamente todos los archivos de modelos de Gazebo
def find_model_files(directory):
    paths = []
    if os.path.exists(directory):
        for (path, directories, filenames) in os.walk(directory):
            for filename in filenames:
                paths.append(os.path.join(path, filename))
    return paths

# Agrupar archivos por su directorio de destino
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    (os.path.join('share', package_name, 'config'), glob('gemelo_digital_qav250/config/*.yaml')),
    (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
]

# Agregar archivos de modelos a la instalación de ROS 2 share
model_files = find_model_files('models')
for model_file in model_files:
    rel_dir = os.path.dirname(model_file)
    dest_dir = os.path.join('share', package_name, rel_dir)
    data_files.append((dest_dir, [model_file]))

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SOPORTE',
    maintainer_email='support@example.com',
    description='Digital Twin for the QAV250 Quadcopter on FFT GYRO test stand',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nodo_gemelo_digital = gemelo_digital_qav250.nodo_gemelo_digital:main',
            'captura_pwm = gemelo_digital_qav250.captura_pwm:main',
            'rutina_demo = gemelo_digital_qav250.rutina_demo:main',
            'ver_pose = gemelo_digital_qav250.ver_pose:main',
            'registrar_datos = gemelo_digital_qav250.registrar_datos:main',
        ],
    },
)
