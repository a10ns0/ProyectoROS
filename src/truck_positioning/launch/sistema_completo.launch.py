from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('truck_positioning')

    # 1. Arrancar Sensor ANTIGUO (Estructura - Origen)
    # Este archivo ya incluye su propia TF en (0,0,3)
    sensor_antiguo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'sensor_antiguo.launch.py') 
        ),
        launch_arguments={
            'hostname': '192.168.1.101',
        }.items()
    )

    # 2. Arrancar Sensor LONGITUDINAL (Distancia)
    # Este archivo ya incluye su propia TF en (5,0,3)
    sensor_longitudinal = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'sensor_longitudinal.launch.py')
        ),
        launch_arguments={
            'hostname': '192.168.1.100',
        }.items()
    )
    
    # 3. Arrancar el nodo calculador de distancia
    calculador_nodo = Node(
        package='truck_positioning',
        executable='calculador_distancia',
        output='screen'
    )

    return LaunchDescription([
        sensor_antiguo,
        sensor_longitudinal,
        calculador_nodo
    ])
