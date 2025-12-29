import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests
import json

class GruaFullStateClient(Node):
    def __init__(self):
        super().__init__('grua_full_state_client')
        
        # --- CONFIGURACIÓN ---
        self.ip_servidor = "192.168.1.88"
        self.port = "8000"
        self.nombre_grua = "STS-005"
        
        # Lista de variables a consultar
        self.variables_a_consultar = ["trolleyPos", "spreaderSize", "spreaderTwistlock"]
        
        self.publisher_ = self.create_publisher(String, 'grua/estado_completo', 10)
        
        # Timeout seguro para redes inestables
        self.timeout_requests = 0.5
        
        self.timer_period = 0.5 
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.get_logger().info(f'Monitor de Grúa iniciado. Objetivo: {self.ip_servidor}')

    def obtener_dato_individual(self, variable):
        """Consulta la API y extrae SOLO el valor limpio"""
        url = f"http://{self.ip_servidor}:{self.port}/gruas/{self.nombre_grua}/{variable}"
        try:
            response = requests.get(url, timeout=self.timeout_requests) 
            response.raise_for_status()
            
            # La API devuelve: {"trolleyPos": "PARKING"}
            json_crudo = response.json()
            
            # --- CORRECCIÓN 1: Extraer el valor limpio ---
            # Buscamos la llave 'variable' dentro del JSON. Si no existe, devolvemos el JSON entero.
            valor_limpio = json_crudo.get(variable, json_crudo)
            
            return valor_limpio
            
        except requests.exceptions.RequestException as e:
            self.get_logger().warn(f"Fallo al leer {variable}: {e}")
            return None

    def timer_callback(self):
        estado_grua = {}
        
        for var in self.variables_a_consultar:
            valor = self.obtener_dato_individual(var)
            
            # --- CORRECCIÓN 2: Lógica de Seguridad para Números (Opcional) ---
            # Si necesitas hacer cálculos matemáticos con 'trolleyPos', 
            # descomenta esto para convertir "PARKING" en un número seguro (ej. 0.0)
            
            # if var == "trolleyPos" and valor == "PARKING":
            #     valor = 0.0  # Asumimos posición 0 si está estacionada
            
            estado_grua[var] = valor

        # Solo publicamos si obtuvimos datos válidos (evita enviar JSONs vacíos)
        if any(v is not None for v in estado_grua.values()): 
            msg = String()
            msg.data = json.dumps(estado_grua)
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GruaFullStateClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
