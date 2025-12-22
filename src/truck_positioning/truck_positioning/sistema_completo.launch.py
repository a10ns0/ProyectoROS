from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('truck_positioning')

    # 1. Arrancar Sensor ANTIGUO (Amarillo)
    sensor_antiguo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            # ASEGÚRATE QUE ESTE SEA EL NOMBRE CORRECTO DE TU ARCHIVO DEL SENSOR VIEJO
            os.path.join(pkg_path, 'sensor_antiguo.launch.py') 
        ),
        launch_arguments={
            'hostname': '192.168.0.2',
            'frame_id': 'sensor_antiguo_link' # Forza el nombre del frame
        }.items()
    )

    # 2. Arrancar Sensor NUEVO (Rosado)
    sensor_nuevo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
             # ASEGÚRATE QUE ESTE SEA EL NOMBRE CORRECTO DE TU ARCHIVO DEL SENSOR NUEVO
            os.path.join(pkg_path, 'sensor_longitudinal.launch.py')
        ),
        launch_arguments={
            'hostname': '192.168.0.3',
            'frame_id': 'sensor_nuevo_link' # Forza el nombre del frame
        }.items()
    )

    # =====================================================================
    # AQUÍ ES DONDE SE DEFINE LA POSICIÓN FÍSICA EN LA GRÚA (TF)
    # Los números son: x y z yaw pitch roll
    # x: Distancia hacia adelante (metros)
    # z: Altura (metros)
    # pitch: Rotación hacia abajo (radianes). 1.5708 son 90 grados (mirando al suelo)
    # =====================================================================

    # Transformada para el Sensor ANTIGUO (Digamos que está en el origen, x=0)
    tf_antiguo = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_antiguo_broadcaster',
        # Argumentos: x y z yaw pitch roll frame_padre frame_hijo
        # EJEMPLO: Está en x=0, a 3 metros de altura (z=3), mirando hacia abajo (pitch=1.57)
        arguments=['0', '0', '3.0', '0', '1.5708', '0', 'map', 'sensor_antiguo_link']
    )

    # Transformada para el Sensor NUEVO (Digamos que está a 5 metros del otro)
    tf_nuevo = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_nuevo_broadcaster',
        # EJEMPLO: Está en x=5.0 (5 metros adelante), misma altura y rotación.
        # --- ¡AJUSTA EL '5.0' PARA CAMBIAR LA DISTANCIA ENTRE SENSORES! ---
        arguments=['5.0', '0', '3.0', '0', '1.5708', '0', 'map', 'sensor_nuevo_link']
    )
    # =====================================================================

    # 4. Arrancar el nodo calculador de distancia
    calculador_nodo = Node(
        package='truck_positioning',
        executable='calculador_distancia',
        output='screen'
    )

    # 5. Arrancar RViz2 automáticamente con una configuración preguardada
    # (Opcional, primero probaremos sin esto)
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', os.path.join(pkg_path, 'config_camion.rviz')]
    # )

    return LaunchDescription([
        sensor_antiguo,
        sensor_nuevo,
        tf_antiguo,
        tf_nuevo,
        calculador_nodo,
        # rviz_node
    ])
