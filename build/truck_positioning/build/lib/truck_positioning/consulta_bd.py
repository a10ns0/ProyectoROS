import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class GruaVisualizer(Node):
    def __init__(self):
        super().__init__('grua_visualizer')
        
        self.subscription = self.create_subscription(
            String,
            'grua/spreader_twistlock',
            self.listener_callback,
            10)
        self.get_logger().info('Esperando datos de la grúa...')

    def listener_callback(self, msg):
        try:
            # 1. Convertimos el String de ROS vuelta a un Diccionario Python
            data_json = json.loads(msg.data)
            
            # 2. Mostramos la información
            # Aquí puedes acceder a las claves del JSON directamente
            # Por ejemplo, si la respuesta es {"estado": true, "valor": 1}
            print(f"--- Recibido ---")
            print(f"Datos crudos: {data_json}")
            
            # Ejemplo de acceso a valor (ajustar según la respuesta real de tu API)
            # valor = data_json.get('value') 
            # print(f"Estado Twistlock: {valor}")

        except json.JSONDecodeError:
            self.get_logger().error('Error al decodificar el JSON recibido')

def main(args=None):
    rclpy.init(args=args)
    node = GruaVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()