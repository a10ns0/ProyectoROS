import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    sick_scan_pkg_prefix = get_package_share_directory('sick_scan_xd')
    launch_file_auto_restart_params = os.path.join(sick_scan_pkg_prefix, 'config', 'sick_auto_restart.yaml')

    hostname_launch_arg = DeclareLaunchArgument('hostname', default_value='192.168.0.3')
    udp_receiver_ip_launch_arg = DeclareLaunchArgument('udp_receiver_ip', default_value='0.0.0.0')

    # 1. NODO DEL SENSOR (Driver "Amordazado")
    sick_scan_node = Node(
        package='sick_scan_xd',
        executable='sick_generic_caller',
        name='sick_scan_xd_longitudinal',
        output='screen',
        parameters=[
            launch_file_auto_restart_params,
            {
                'scanner_type': 'sick_lms_5xx',
                'hostname': LaunchConfiguration('hostname'),
                'udp_receiver_ip': LaunchConfiguration('udp_receiver_ip'),
                'port': '2112',
                'lfe_msg_subscriber': False,
                'min_ang': -1.658,
                'max_ang': 1.658,
                'use_binary_protocol': True,
                'intensity': True,
                'range_max': 2.5,
                'frame_id': 'cloud_longitudinal'
            }
        ],
        remappings=[
            ('/sick_lms_5xx/scan', '/scan_distancia'),
            # --- EL CORTE DE CABLE ---
            ('/tf', '/tf_ignorado_long'),          # Enviamos su TF a la basura
            ('/tf_static', '/tf_static_ignorado')  # También la estática
            # -------------------------
        ]
    )

    # 2. NODO DE TRANSFORMADA (Manual y Autorizado)
    tf_sensor_longitudinal = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_sensor_longitudinal',
        arguments=['1', '0', '1.5', '1.57', '1.57', '0', 'base_link', 'cloud_longitudinal'],
        output='screen'
    )

    return LaunchDescription([
        hostname_launch_arg, 
        udp_receiver_ip_launch_arg, 
        sick_scan_node, 
        tf_sensor_longitudinal
    ])
