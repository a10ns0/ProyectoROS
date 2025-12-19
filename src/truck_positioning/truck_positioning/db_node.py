# src/truck_positioning/truck_positioning/db_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

# Importamos la clase que creamos arriba (nota el punto antes de database_manager)
from .database_manager import Database

class CraneDBNode(Node):
    def __init__(self):
        super().__init__('crane_db_node')
        self.publisher_ = self.create_publisher(String, '/crane/config', 10)
        
        # Timer para consultar cada 1 segundo (1.0)
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.db = Database()
        self.id_grua = 1 # ID de la grúa actual
        self.get_logger().info('Nodo DB Iniciado. Conectando a SQL Server...')

    def timer_callback(self):
        # 1. Consultar DB
        datos = self.db.obtener_configuracion_grua(self.id_grua)
        
        if datos:
            # 2. Crear diccionario con los datos
            # Asumimos que la DB devuelve: [0]=angulo, [1]=distancia, [2]=estado
            config_dict = {
                "angulo": float(datos[0]),
                "limite": float(datos[1]),
                "estado": str(datos[2])
            }
            
            # 3. Convertir a JSON String y publicar
            msg = String()
            msg.data = json.dumps(config_dict)
            self.publisher_.publish(msg)
            
            # self.get_logger().info(f'Publicando: {msg.data}') # Descomentar para debug
        else:
            self.get_logger().warning('No se pudieron leer datos de la DB', throttle_duration_sec=5)

def main(args=None):
    rclpy.init(args=args)
    node = CraneDBNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()