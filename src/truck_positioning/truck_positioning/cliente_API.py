import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests
import json

class GruaApiClient(Node):
    def __init__(self):
        super().__init__('grua_api_client')
        
        # CONFIGURACIÓN
        self.ip_servidor = "192.168.1.88" # IP Real entregada
        self.port = "8000"
        self.nombre_grua = "STS-001"      # Según tu URL
        self.variable_objetivo = "spreaderTwistlock" # Variable a consultar
        
        # Publisher
        self.publisher_ = self.create_publisher(String, 'grua/spreader_twistlock', 10)
        
        # Timer (Polling) - Ajustable según necesidad (0.5s = 2Hz)
        self.timer_period = 0.5  
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.get_logger().info(f'Iniciando consulta a {self.ip_servidor} para {self.variable_objetivo}')

    # --- FUNCIÓN INTEGRADA DEL SCRIPT QUE TE PASARON ---
    def obtener_datos_grua(self, nombre_grua, variable):
        url = f"http://{self.ip_servidor}:{self.port}/gruas/{nombre_grua}/{variable}"
        try:
            # Agregamos timeout=1.0 para seguridad en tiempo real (evita bloqueos)
            response = requests.get(url, timeout=1.0)
            response.raise_for_status()
            data = response.json()
            return data
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Error API: {e}")
            return None
    # ---------------------------------------------------

    def timer_callback(self):
        # 1. Obtenemos el dato usando la función integrada
        datos = self.obtener_datos_grua(self.nombre_grua, self.variable_objetivo)
        
        if datos is not None:
            # 2. Empaquetamos el JSON en un String para enviarlo por ROS
            msg = String()
            msg.data = json.dumps(datos) # Convertimos dict a string JSON
            
            # 3. Publicamos
            self.publisher_.publish(msg)
            # self.get_logger().info(f'Publicado: {msg.data}') # Descomentar para debug

def main(args=None):
    rclpy.init(args=args)
    node = GruaApiClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()