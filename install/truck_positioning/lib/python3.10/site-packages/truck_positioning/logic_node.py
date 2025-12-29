#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import numpy as np

# Mensajes necesarios
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
# Importamos el mensaje específico de SICK para leer los "bloques"
from sick_scan_xd.msg import LFErecMsg 

class TruckLogicNode(Node):
    def __init__(self):
        super().__init__('truck_logic_node')

        # --- CONFIGURACIÓN ---
        self.limit_distance_x = 5.0  # El límite donde queremos que pare el camión (metros)
        
        # --- SUSCRIPTORES (LISTENERS) ---
        # 1. Escuchar los datos del láser (para calcular distancia precisa)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # 2. Escuchar los "Bloques" de SICK (LFErec)
        self.create_subscription(LFErecMsg, '/sick_scan/lferec', self.field_callback, 10)

        # --- PUBLICADORES (PUBLISHERS) ---
        # 1. Visualización de los bloques en RViz (Cajas virtuales)
        self.marker_pub = self.create_publisher(MarkerArray, '/viz/blocks', 10)
        # 2. Texto para el operador (El "Semáforo" virtual)
        self.text_pub = self.create_publisher(Marker, '/viz/feedback_text', 10)

        self.get_logger().info('Sistema de Posicionamiento Iniciado')

    def field_callback(self, msg):
        """
        Se ejecuta cuando el SICK reporta el estado de los campos (Bloques).
        msg.fields contiene el estado: 0=Desactivado, 1=Libre, 2=Ocupado (Infringido)
        """
        markers = MarkerArray()
        
        for i, field in enumerate(msg.fields):
            # Creamos un cubo visual para representar el bloque en RViz
            marker = Marker()
            marker.header.frame_id = "cloud" # El marco de referencia del láser
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "sick_fields"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            # SIMULACION VISUAL: Como ROS no sabe dónde dibujaste los campos en SOPAS,
            # ponemos cubos genéricos a distintas distancias para representarlos visualmente.
            # Ajusta estas posiciones (position.x) para que coincidan con tu realidad.
            if i == 0: 
                marker.pose.position.x = 4.5 # Bloque de STOP
                marker.scale.x = 1.0
            else:
                marker.pose.position.x = 10.0 # Bloque de Advertencia
                marker.scale.x = 5.0
                
            marker.pose.position.y = 0.0
            marker.pose.position.z = 0.0
            marker.scale.y = 3.0 # Ancho del campo
            marker.scale.z = 1.0
            
            # LOGICA DE COLOR: Verde si está libre, Rojo si el camión lo toca
            if field.field_result_mrs == 2: # 2 = Infringido/Ocupado
                marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.5) # Rojo semitransparente
            else:
                marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.2) # Verde transparente
                
            markers.markers.append(marker)
        
        self.marker_pub.publish(markers)

    def scan_callback(self, msg):
        """
        Aquí aplicamos PITÁGORAS para calcular la distancia exacta al camión.
        """
        # Convertimos los rangos polares a coordenadas X, Y
        ranges = np.array(msg.ranges)
        # Generamos los ángulos correspondientes a cada rango
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        
        # Filtramos datos inválidos (infinito o 0)
        valid_idx = np.where((ranges > 0.1) & (ranges < 50.0))
        ranges = ranges[valid_idx]
        angles = angles[valid_idx]

        if len(ranges) == 0: return

        # --- MATEMÁTICA BÁSICA (Polar a Cartesiana) ---
        # x = distancia * cos(angulo)
        # y = distancia * sen(angulo)
        x_coords = ranges * np.cos(angles)
        y_coords = ranges * np.sin(angles)

        # Filtramos solo lo que está frente al sensor (en el carril del camión)
        # Supongamos que el carril mide 3 metros de ancho (Y entre -1.5 y 1.5)
        lane_filter = np.where((y_coords > -1.5) & (y_coords < 1.5))
        
        if len(lane_filter) > 0:
            # Tomamos las coordenadas X de los puntos dentro del carril
            truck_points_x = x_coords[lane_filter]
            
            # La distancia al camión es el punto X más pequeño (la parte frontal del camión)
            truck_dist = np.min(truck_points_x)
            
            # --- CÁLCULO DE RETROALIMENTACIÓN ---
            delta = truck_dist - self.limit_distance_x
            
            if abs(delta) < 0.3: # Tolerancia de 30cm
                self.publish_text("¡STOP! - EN POSICIÓN", [0.0, 1.0, 0.0]) # Verde
            elif delta > 0:
                self.publish_text(f"AVANZA {delta:.2f} m", [1.0, 1.0, 0.0]) # Amarillo
            else:
                self.publish_text(f"RETROCEDE {abs(delta):.2f} m", [1.0, 0.0, 0.0]) # Rojo
        else:
            self.publish_text("ESPERANDO CAMIÓN...", [1.0, 1.0, 1.0])

    def publish_text(self, text, color_list):
        marker = Marker()
        marker.header.frame_id = "cloud"
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 2.0 # Texto flotando 2m arriba del sensor
        marker.scale.z = 0.5 # Tamaño letra
        marker.color = ColorRGBA(r=color_list, g=color_list[1], b=color_list[2], a=1.0)
        marker.text = text
        self.text_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = TruckLogicNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()