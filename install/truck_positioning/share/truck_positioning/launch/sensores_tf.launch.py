from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # --- SENSOR 1 (IZQUIERDA) ---
        # Sintaxis: x y z yaw pitch roll parent_frame child_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_sensor_izq',
            arguments=['0', '0', '2.0', '0', '0', '0', 'base_camion', 'laser_izq']
            # Explicación:
            # 0.5 metros hacia adelante (X)
            # 2.0 metros hacia la izquierda (Y)
            # 3.0 metros de altura (Z)
            # 'base_camion' es el centro del camión
            # 'laser_izq' es el frame_id que anotaste de tu sensor 1
        ),

        # --- SENSOR 2 (DERECHA) ---
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_sensor_der',
            arguments=['0.5', '0', '0.1', '0', '0', '1.57', 'base_camion', 'laser_der']
            # Explicación:
            # -2.0 metros (negativo) significa 2m a la DERECHA
            # 'laser_der' es el frame_id que anotaste de tu sensor 2
        ),
    ])
