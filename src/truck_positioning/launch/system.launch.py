from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        #1. Nodo del driver (con remapeo si es necesario)
        Node(
            package='sick_scan_xd',
            executable='sick_generic_caller',
            name='sick_driver',
            output='screen',
            remappings=[('/sick_scan/lferec', '/sick_scan/lferec')]
        ),
                
        
        # 2. Tu nodo de lógica
        Node(
            package='truck_positioning',
            executable='logic_node',
            name='logic_controller',
            output='screen'
        ),
        
        # 3. RViz para ver todo
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            # Aquí se podría cargar un archivo.rviz preconfigurado
        )
    ])