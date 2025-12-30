import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster
import numpy as np
import math

class TruckSimulator(Node):
    def __init__(self):
        super().__init__('truck_simulator')
        
        # Publicador de TF (Para mover el modelo en RViz)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # hacemos match con los topicos que escucha el visualizador_propio
        self.pub_scan1 = self.create_publisher(LaserScan, '/scan_estructura', 10)
        self.pub_scan2 = self.create_publisher(LaserScan, '/scan_distancia', 10)
        
        # Estado del Camión
        self.truck_x = -15.0 # Empieza lejos
        self.velocity = 2.0  # m/s (aprox 7 km/h)
        self.dt = 0.05       # 20 Hz
        
        # Configuración Física Simulada
        self.sensor1_pos_x = 0.0
        self.sensor2_pos_x = 22.38
        self.sensor_height = 12.5
        
        # Dimensiones del Camión Simulado
        self.truck_length = 13.7
        self.cabin_length = 2.5
        self.chassis_height = 1.2
        self.cabin_height = 3.0
        
        self.timer = self.create_timer(self.dt, self.update_simulation)
        self.get_logger().info("Simulador de Camión Iniciado. Mira RViz.")

    def get_fake_reading(self, sensor_x):
        """Calcula qué vería el sensor basado en la posición actual del camión"""
        # Distancia relativa: ¿En qué parte del camión está el sensor?
        # (Asumiendo que truck_x es la COLA del camión)
        relative_pos = sensor_x - self.truck_x
        
        detected_height = 0.0 # Suelo
        
        if 0 <= relative_pos <= self.truck_length:
            # El sensor está sobre el camión
            if relative_pos > (self.truck_length - self.cabin_length):
                detected_height = self.cabin_height # Zona Cabina
            else:
                detected_height = self.chassis_height # Zona Chasis
        
        # Convertir altura a distancia medida por el sensor
        measured_range = self.sensor_height - detected_height
        
        # Agregar un poco de ruido (Simulación realista)
        noise = np.random.normal(0, 0.02) # +/- 2cm de ruido
        return measured_range + noise

    def create_scan_msg(self, range_val, frame_id):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.angle_min = -0.1
        msg.angle_max = 0.1
        msg.angle_increment = 0.1
        msg.range_min = 0.0
        msg.range_max = 100.0
        # Creamos un scan de 3 puntos simulados
        msg.ranges = [range_val, range_val, range_val] 
        return msg

    def update_simulation(self):
        # 1. Física: Mover el camión
        if self.truck_x < 30.0:
            self.truck_x += self.velocity * self.dt
        else:
            self.truck_x = -15.0 # Resetear simulación (Loop infinito)

        # 2. Visualización: Publicar TF para RViz
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'       # El mundo
        t.child_frame_id = 'base_link'  # El camión
        t.transform.translation.x = self.truck_x
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0    # Sin rotación
        self.tf_broadcaster.sendTransform(t)

        # 3. Sensores: Generar datos falsos
        range1 = self.get_fake_reading(self.sensor1_pos_x)
        range2 = self.get_fake_reading(self.sensor2_pos_x)
        
        self.pub_scan1.publish(self.create_scan_msg(range1, 'sensor1_link'))
        self.pub_scan2.publish(self.create_scan_msg(range2, 'sensor2_link'))

def main(args=None):
    rclpy.init(args=args)
    node = TruckSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
