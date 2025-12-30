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
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pub_scan1 = self.create_publisher(LaserScan, '/scan_estructura', 10)
        self.pub_scan2 = self.create_publisher(LaserScan, '/scan_distancia', 10)
        
        # --- FÍSICA ---
        self.truck_x = -15.0 
        self.velocity = 2.0   
        self.dt = 0.05        
        
        # --- POSICIÓN DE SENSORES ---
        self.sensor1_pos_x = 0.0     # Estructura
        self.sensor2_pos_x = 22.38   # Longitudinal
        self.sensor_height = 12.5    # Altura del pórtico
        
        # --- HITBOXES (CAJAS DE COLISIÓN) ---
        # Coordenadas locales relativas a la cola del camión (0.0)
        self.limit_truck_start = 0.0
        self.limit_truck_end = 12.0
        
        # La cabina está al final (9.5m a 12.0m)
        self.limit_cabin_start = 9.5
        
        self.h_chassis = 1.2
        self.h_cabin = 3.0
        
        self.timer = self.create_timer(self.dt, self.update_simulation)
        self.get_logger().info(">>> SIMULADOR: LÓGICA DE SOMBRA (SHADOW CAST) <<<")

    def get_smart_distance(self, sensor_x, angle_rad):
        """
        Lógica 'Shadow Casting':
        1. Proyectamos el láser al SUELO.
        2. Vemos si el camión está 'pisando' ese punto.
        3. Si está, corregimos la altura.
        """
        # A. Proyectar al suelo (Z=0)
        # Tan(theta) = Opuesto(dx) / Adyacente(Altura Sensor)
        dx_floor = self.sensor_height * math.tan(angle_rad)
        
        # Coordenada absoluta en el mundo donde el rayo tocaría el piso
        hit_x_world = sensor_x + dx_floor
        
        # B. Convertir a coordenadas locales del camión
        # (Para saber en qué parte del camión cayó el rayo)
        x_local = hit_x_world - self.truck_x
        
        # C. Determinar Altura del Objeto en ese punto
        detected_height = 0.0 # Por defecto: Suelo
        
        if self.limit_truck_start <= x_local <= self.limit_truck_end:
            # Estamos dentro de la huella del camión
            if x_local >= self.limit_cabin_start:
                detected_height = self.h_cabin   # Cabina
            else:
                detected_height = self.h_chassis # Chasis
        
        # D. Calcular la distancia real (Hipotenusa) al sensor
        # Distancia = (Altura Sensor - Altura Detectada) / cos(theta)
        
        vertical_dist = self.sensor_height - detected_height
        
        # Protección contra división por cero
        if abs(math.cos(angle_rad)) < 0.01:
            return 20.0 # Rayo horizontal infinito
            
        r = vertical_dist / math.cos(angle_rad)
        return r

    def create_scan_msg(self, frame_id, sensor_x, fov_min, fov_max, num_points=100):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        
        # Conversión a radianes
        angle_min_rad = np.radians(fov_min)
        angle_max_rad = np.radians(fov_max)
        
        msg.angle_min = angle_min_rad
        msg.angle_max = angle_max_rad
        msg.angle_increment = (angle_max_rad - angle_min_rad) / (num_points - 1)
        msg.range_min = 0.0
        msg.range_max = 100.0
        
        ranges = []
        
        for i in range(num_points):
            current_angle = angle_min_rad + (i * msg.angle_increment)
            
            # Solo calculamos ángulos físicos válidos (mirando hacia abajo)
            # +/- 88 grados
            if -1.55 < current_angle < 1.55:
                r = self.get_smart_distance(sensor_x, current_angle)
                
                # Ruido pequeño (+/- 1cm) para realismo
                noise = np.random.normal(0, 0.01)
                ranges.append(r + noise)
            else:
                ranges.append(0.0) # Fuera de rango
            
        msg.ranges = ranges
        return msg

    def update_simulation(self):
        # A. FÍSICA
        if self.truck_x < 35.0:
            self.truck_x += self.velocity * self.dt
        else:
            self.truck_x = -15.0

        # B. VISUALIZACIÓN
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.truck_x
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

        # C. SENSORES
        # Sensor Estructura (Barrido Ancho +/- 45)
        msg1 = self.create_scan_msg('sensor1_link', self.sensor1_pos_x, -45, 45, 100)
        self.pub_scan1.publish(msg1)
        
        # Sensor Longitudinal (Barrido Largo -85 a 0)
        msg2 = self.create_scan_msg('sensor2_link', self.sensor2_pos_x, -85, 0, 100)
        self.pub_scan2.publish(msg2)

def main(args=None):
    rclpy.init(args=args)
    node = TruckSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
