import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import numpy as np

class TruckPerceptionNode(Node):
    def __init__(self):
        super().__init__('truck_perception_node')
        
        # --- CONFIGURACIÓN FÍSICA ---
        self.SENSOR_HEIGHT = 12.5       # Altura de montaje (metros)
        self.GROUND_THRESHOLD = 11.8    # Distancia a partir de la cual se considera suelo
        
        # --- SUSCRIPTORES ---
        # Sensor 1: Entrada (Top-Down). Origen X=0.0
        self.sub_scan1 = self.create_subscription(LaserScan, '/sick_entry/scan', self.scan1_callback, 10)
        # Sensor 2: Fondo (Top-Down). Origen X=22.38
        self.sub_scan2 = self.create_subscription(LaserScan, '/sick_end/scan', self.scan2_callback, 10)
        
        # --- PUBLICADORES ---
        # 1. Posición calculada de la cola del camión (Input para el nodo de Control)
        self.pub_tail_pos = self.create_publisher(Float32, '/tps/truck_tail_position', 10)
        
        # 2. Visualización DEBUG (Perfil de altura en tiempo real)
        # Grafica esto en rqt_plot para ver la forma del camión pasando
        self.pub_debug_profile = self.create_publisher(Float32, '/tps/debug/height_profile', 10)
        
        # Buffers de puntos
        self.points_scan1 = np.array([])
        self.points_scan2 = np.array([])
        
        # Timer de procesamiento (20 Hz)
        self.timer = self.create_timer(0.05, self.process_fusion_and_publish)

        self.get_logger().info(f'Percepción Iniciada. Altura Sensores: {self.SENSOR_HEIGHT}m')

    def process_scan(self, msg, offset_x):
        """ Convierte LaserScan polar a Coordenadas X Lineales filtrando el suelo """
        ranges = np.array(msg.ranges)
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        
        # 1. Filtro de Validez
        valid_mask = np.isfinite(ranges) & (ranges > 0.5)
        r = ranges[valid_mask]
        theta = angles[valid_mask]
        
        # 2. Filtro de Altura (ELIMINACIÓN DE SUELO)
        # Si r > GROUND_THRESHOLD (11.8m), es suelo. Lo descartamos.
        object_mask = r < self.GROUND_THRESHOLD
        r_obj = r[object_mask]
        theta_obj = theta[object_mask]
        
        if len(r_obj) == 0:
            return np.array([]), 0.0
            
        # 3. Cálculo de Altura Máxima en este scan (Para visualización)
        # La altura del objeto es la altura del sensor menos la distancia medida
        min_dist = np.min(r_obj)
        max_height_detected = self.SENSOR_HEIGHT - min_dist
        
        # 4. Proyección a Eje X (Longitudinal)
        # Asumiendo montaje con 0 grados apuntando abajo y barrido longitudinal
        x_local = r_obj * np.cos(theta_obj) # O np.sin dependiendo de la rotación del sensor
        x_global = x_local + offset_x
        
        return x_global, max_height_detected

    def scan1_callback(self, msg):
        # Procesa Sensor 1 y extrae perfil para visualización
        self.points_scan1, height_val = self.process_scan(msg, offset_x=0.0)
        
        # Publicamos el perfil de altura INMEDIATAMENTE para visualización fluida
        debug_msg = Float32()
        debug_msg.data = float(height_val)
        self.pub_debug_profile.publish(debug_msg)

    def scan2_callback(self, msg):
        self.points_scan2, _ = self.process_scan(msg, offset_x=22.38)

    def process_fusion_and_publish(self):
        # Fusión de los dos sensores
        if len(self.points_scan1) == 0 and len(self.points_scan2) == 0:
            return # No hay camión

        all_points = np.concatenate([self.points_scan1, self.points_scan2])
        
        if len(all_points) < 5: 
            return # Ruido

        # DETECCIÓN DE LA COLA (Borde Trasero)
        # Usamos percentil 1 para robustez contra puntos espurios
        tail_x = np.percentile(all_points, 1)
        
        # Publicar posición
        msg = Float32()
        msg.data = float(tail_x)
        self.pub_tail_pos.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TruckPerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()