import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    sick_scan_pkg_prefix = get_package_share_directory('sick_scan_xd')
    launch_file_auto_restart_params = os.path.join(sick_scan_pkg_prefix, 'config', 'sick_auto_restart.yaml')

    # 1. NODO DEL SENSOR (Driver "Amordazado")
    sick_scan_node = Node(
        package='sick_scan_xd',
        executable='sick_generic_caller',
        name='sick_scan_sensor_antiguo',
        output='screen',
        parameters=[
            launch_file_auto_restart_params,
            {
                'scanner_type': 'sick_lms_5xx',
                'hostname': '192.168.0.2',
                'port': '2112',
                'udp_receiver_ip': '0.0.0.0',
                'min_ang': -1.57,
                'max_ang': 1.57,
                'range_max': 25.0,
                'frame_id': 'sensor_antiguo_link',
                'intensity': True,
                'intensity_resolution_16bit': False,
                'use_binary_protocol': True,
                'timelimit': 5
            }
        ],
        remappings=[
            ('/sick_lms_5xx/scan', '/scan_estructura'),
            # --- EL CORTE DE CABLE ---
            ('/tf', '/tf_ignorado_antiguo'),       # Enviamos su TF a la basura
            ('/tf_static', '/tf_static_ignorado')  # También la estática
            # -------------------------
        ]
    )

    # 2. NODO DE TRANSFORMADA (Manual y Autorizado)
    tf_sensor_antiguo = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_sensor_antiguo',
        arguments=['0', '0', '1.5', '0', '1.57', '0', 'base_link', 'sensor_antiguo_link'],
        output='screen'
    )

    return LaunchDescription([sick_scan_node, tf_sensor_antiguo])
