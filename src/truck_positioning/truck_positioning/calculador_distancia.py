#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
import numpy as np

class CalculadorDistancia(Node):
    def __init__(self):
        super().__init__('calculador_distancia')
        # Escuchamos al sensor NUEVO (el que suele estar al final para medir distancia)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan_longitudinal',
            self.listener_callback,
            10)
        # Publicamos un marcador de texto 3D para RViz
        self.publisher_ = self.create_publisher(Marker, '/distancia_texto', 10)
        self.get_logger().info('Calculador de distancia iniciado...')

    def listener_callback(self, msg):
        # Encontramos el punto más cercano en el tercio central del escáner
        # (Asumiendo que el sensor mira hacia abajo, el centro es el suelo/camión)
        num_puntos = len(msg.ranges)
        inicio_centro = int(num_puntos / 3)
        fin_centro = int(num_puntos * 2 / 3)
        
        rangos_centrales = np.array(msg.ranges[inicio_centro:fin_centro])
        
        # Filtramos ceros e infinitos
        validos = (rangos_centrales > msg.range_min) & (rangos_centrales < msg.range_max)
        rangos_limpios = rangos_centrales[validos]

        if len(rangos_limpios) > 0:
            distancia_minima = np.min(rangos_limpios)
            self.publicar_marcador(distancia_minima, msg.header.frame_id)
        else:
            self.publicar_marcador(0.0, msg.header.frame_id, "Sin deteccion")

    def publicar_marcador(self, distancia, frame_id, texto_extra=""):
        marker = Marker()
        marker.header.frame_id = "map" # Referencia global
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        # Posición del texto en el espacio 3D (ajústalo si sale muy arriba/abajo)
        marker.pose.position.x = 2.5 # A mitad de camino entre los sensores
        marker.pose.position.y = 0.0
        marker.pose.position.z = 2.0 # Altura del texto

        marker.scale.z = 0.5 # Tamaño de la letra

        if texto_extra:
            marker.text = texto_extra
            marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0 # Rojo
        else:
            # Aquí calculamos "cuánto falta". Supongamos que la meta es estar a 1.5m
            meta = 1.5
            falta = distancia - meta
            if falta < 0: falta = 0
            marker.text = f"Distancia al objetivo: {falta:.2f} m\n(Medicion actual: {distancia:.2f} m)"
            marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 0.0 # Amarillo

        marker.color.a = 1.0 # Transparencia

        self.publisher_.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = CalculadorDistancia()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
