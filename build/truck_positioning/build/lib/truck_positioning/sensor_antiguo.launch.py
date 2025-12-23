import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Ruta al archivo de configuración básico (usamos el por defecto)
    sick_scan_pkg_prefix = get_package_share_directory('sick_scan_xd')
    launch_file_auto_restart_params = os.path.join(sick_scan_pkg_prefix, 'config', 'sick_auto_restart.yaml')

    # Nodo del Sensor ANTIGUO (Amarillo)
    sick_scan_node = Node(
        package='sick_scan_xd',
        executable='sick_generic_caller',
        name='sick_scan_sensor_antiguo',
        output='screen',
        parameters=[
            launch_file_auto_restart_params, # Carga parametros base
            {
                'scanner_type': 'sick_lms_5xx',
                
                # --- CONFIGURACIÓN DE RED ---
                'hostname': '192.168.0.2',  # IP del sensor viejo
                'port': '2112',
                'udp_receiver_ip': '0.0.0.0',
                
                # --- ÁNGULOS Y RANGO ---
                'min_ang': -1.57,  # Coma obligatoria al final
                'max_ang': 1.57,   # Coma obligatoria al final
                'range_max': 25.0,
                
                # --- IDENTIFICACIÓN ---
                'frame_id': 'sensor_antiguo_link', # ¡Vital para el sistema 3D!
                
                # --- EXTRAS ---
                'intensity': True,
                'intensity_resolution_16bit': False,
                'use_binary_protocol': True,
                'timelimit': 5
            }
        ],
        remappings=[
            ('/sick_lms_5xx/scan', '/scan_estructura') # Mantiene el nombre '/scan' original
        ]
    )

    return LaunchDescription([
        sick_scan_node
    ])
