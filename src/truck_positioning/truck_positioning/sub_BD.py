import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class GruaFullVisualizer(Node):
    def __init__(self):
        super().__init__('grua_full_visualizer')
        
        self.subscription = self.create_subscription(
            String,
            'grua/estado_completo', # Suscrito al nuevo tópico
            self.listener_callback,
            10)
        self.get_logger().info('Esperando telemetría completa de la grúa...')

    def listener_callback(self, msg):
        try:
            data = json.loads(msg.data)
            
            # Limpiamos consola (opcional, para efecto de "panel de control")
            # print("\033c", end="") 
            
            print("========================================")
            print(f"   ESTADO GRÚA STS-001 (Recibido)      ")
            print("========================================")
            
            # Extraemos los valores con seguridad (usando .get por si alguno viene null)
            # Asumiendo que la API devuelve el dato dentro de una llave, o el dato directo.
            # Ajustar según la estructura real de tu JSON.
            
            trolley = data.get("trolleyPos", "N/A")
            size    = data.get("spreaderSize", "N/A")
            lock    = data.get("spreaderTwistlock", "N/A")

            print(f" -> Posición Trolley : {trolley}")
            print(f" -> Tamaño Spreader  : {size}")
            print(f" -> Twistlock        : {lock}")
            print("----------------------------------------")

        except json.JSONDecodeError:
            self.get_logger().error('Error de formato JSON')

def main(args=None):
    rclpy.init(args=args)
    node = GruaFullVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()