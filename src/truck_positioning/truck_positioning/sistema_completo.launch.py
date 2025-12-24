from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('truck_positioning')

    # 1. Arrancar Sensor ANTIGUO (Amarillo) - Tópico: /scan_estructura
    sensor_antiguo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'sensor_antiguo.launch.py') 
        ),
        launch_arguments={
            'hostname': '192.168.0.2',
            # El frame_id ya está definido dentro del archivo como 'sensor_antiguo_link'
        }.items()
    )

    # 2. Arrancar Sensor LONGITUDINAL (Rosado) - Tópico: /scan_distancia
    # Este sensor tiene hardcodeado el nombre 'cloud_longitudinal'
    sensor_longitudinal = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'sensor_longitudinal.launch.py')
        ),
        launch_arguments={
            'hostname': '192.168.0.3',
            # Eliminamos el intento de cambiar el frame_id porque el otro archivo lo ignora
        }.items()
    )

    # =====================================================================
    # CONFIGURACIÓN DE POSICIONES (TF)
    # Aquí definimos dónde está cada sensor respecto al mapa (base)
    # =====================================================================

    # Transformada para el Sensor ANTIGUO
    # Frame: sensor_antiguo_link
    tf_antiguo = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_antiguo_broadcaster',
        # x=0, z=3m de altura. HIJO: sensor_antiguo_link
        arguments=['0', '0', '3.0', '0', '0', '0', 'base_link', 'sensor_antiguo_link']
    )

    # Transformada para el Sensor LONGITUDINAL
    # AQUÍ ESTABA EL ERROR: Cambiamos 'sensor_nuevo_link' por 'cloud_longitudinal'
    tf_longitudinal = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_longitudinal_broadcaster',
        # x=5.0 (5 metros separado del otro). HIJO: cloud_longitudinal
        arguments=['5.0', '0', '3.0', '0', '0', '0', 'base_link', 'cloud_longitudinal']
    )
    # =====================================================================

    # 4. Arrancar el nodo calculador de distancia
    calculador_nodo = Node(
        package='truck_positioning',
        executable='calculador_distancia',
        output='screen'
    )

    return LaunchDescription([
        sensor_antiguo,
        sensor_longitudinal,
        tf_antiguo,
        tf_longitudinal,
        calculador_nodo
    ])

