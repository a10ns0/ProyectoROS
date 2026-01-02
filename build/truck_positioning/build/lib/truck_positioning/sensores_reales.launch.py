# Archivo: sensores_reales.launch.py

#Paso 1: Conectar los Sensores Reales (Launch File)
#Primero, se necesita lanzar los drivers oficiales de SICK. No leer los sockets TCP/IP desde Python "a mano", dejar que ROS lo haga.


from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # --- SENSOR 1: Perfil/Estructura (IP 192.168.1.100) ---
        Node(
            package='sick_scan_xd',
            executable='sick_generic_caller',
            name='sick_estructura',
            output='screen',
            parameters=[{
                'scanner_type': 'sick_lms_1xx',  # Cambia según tu modelo exacto (tim_5xx, lms_1xx, etc)
                'hostname': '192.168.1.100',
                'port': 2112,
                'frame_id': 'cloud_estructura',
                'min_ang': -1.57, # -90 grados
                'max_ang': 1.57   # +90 grados
            }],
            remappings=[('/cloud', '/scan_estructura')] # Renombramos tópico para que calce con tu código
        ),

        # --- SENSOR 2: Longitudinal/Distancia (IP 192.168.1.101) ---
        Node(
            package='sick_scan_xd',
            executable='sick_generic_caller',
            name='sick_longitudinal',
            output='screen',
            parameters=[{
                'scanner_type': 'sick_lms_1xx',
                'hostname': '192.168.1.101',
                'port': 2112,
                'frame_id': 'cloud_longitudinal'
            }],
            remappings=[('/cloud', '/scan_distancia')] # Renombramos tópico
        )
    ])