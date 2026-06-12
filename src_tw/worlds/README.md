# Entornos Simulados (worlds)

Esta carpeta almacena la configuración de los mundos `.world` utilizados para el arranque y estabilización de iluminación, gravedad y plugins de ROS 2.

## Componentes

- `qav250_twin.world`: Configura la rampa de arranque básico con iluminación direccional y planos de colisión. Integra el plugin `gazebo_ros_state` que permite que ROS interactúe con el servicio `/set_entity_state`, un requisito fundamental para que los nodos en Python puedan inyectar la matriz absoluta de posición y rotación ignorando el motor físico de Gazebo.

La gravedad `(0, 0, -9.8)` permanece activa en el escenario, sin embargo, el dron principal sobrescribe su posición cada tick (20 Hz), dictando la dinámica real.
