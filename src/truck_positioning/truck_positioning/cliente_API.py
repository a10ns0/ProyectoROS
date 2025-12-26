import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests
import json
import time

class GruaFullStateClient(Node):
    def __init__(self):
        super().__init__('grua_full_state_client')
        
        # --- CONFIGURACIÓN ---
        self.ip_servidor = "192.168.1.88"
        self.port = "8000"
        self.nombre_grua = "STS-001"
        
        # Lista de variables que queremos consultar a la API
        self.variables_a_consultar = ["trolleyPos", "spreaderSize", "spreaderTwistlock"]
        
        self.publisher_ = self.create_publisher(String, 'grua/estado_completo', 10)
        
        # Frecuencia: 0.5s (2 Hz). 
        # NOTA: Al ser 3 peticiones, el tiempo total de ejecución aumenta ligeramente.
        self.timer_period = 0.5  
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.get_logger().info(f'Monitor de Grúa iniciado. Objetivo: {self.ip_servidor}')

    def obtener_dato_individual(self, variable):
        """Función auxiliar para consultar una sola variable"""
        url = f"http://{self.ip_servidor}:{self.port}/gruas/{self.nombre_grua}/{variable}"
        try:
            # Timeout corto (0.2s) por petición para no bloquear el ciclo si falla una
            response = requests.get(url, timeout=0.2) 
            response.raise_for_status()
            return response.json() # Retorna el contenido (ej: {"value": 45.2})
        except requests.exceptions.RequestException as e:
            self.get_logger().warn(f"Fallo al leer {variable}: {e}")
            return None

    def timer_callback(self):
        # Diccionario donde guardaremos la "foto" completa del estado
        estado_grua = {}
        
        # Recorremos la lista de variables y llenamos el diccionario
        for var in self.variables_a_consultar:
            respuesta = self.obtener_dato_individual(var)
            
            # Asumimos que la API devuelve algo como {"value": X} o directamente el valor.
            # Guardamos el resultado crudo bajo la llave correspondiente.
            estado_grua[var] = respuesta

        # Solo publicamos si obtuvimos al menos un dato (o puedes exigir que estén todos)
        if any(estado_grua.values()): 
            msg = String()
            msg.data = json.dumps(estado_grua)
            self.publisher_.publish(msg)
            # self.get_logger().info(f'Estado enviado: {msg.data}')

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