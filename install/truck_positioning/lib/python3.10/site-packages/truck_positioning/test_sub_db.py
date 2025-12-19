import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class TestSubscriber(Node):
    def __init__(self):
        super().__init__('nodo_prueba_lectura')
        
        # Nos suscribimos al tópico donde publica el nodo de base de datos
        self.subscription = self.create_subscription(
            String,
            '/crane/config',
            self.listener_callback,
            10)
        self.subscription  # evitar warning de variable no usada
        
        self.get_logger().info('--- NODO DE PRUEBA INICIADO: Escuchando /crane/config ---')

    def listener_callback(self, msg):
        try:
            # 1. Decodificar el JSON recibido
            data = json.loads(msg.data)
            
            # 2. Imprimir los valores en consola
            print("\n" + "="*30)
            print(f"📡 DATO RECIBIDO DE SQL SERVER:")
            print(f"   ► Ángulo Apertura : {data['angulo']}°")
            print(f"   ► Límite Distancia: {data['limite']} m")
            print(f"   ► Estado Operativo: {data['estado']}")
            print("="*30)
            
        except json.JSONDecodeError:
            self.get_logger().error('El mensaje recibido no es un JSON válido.')
        except KeyError as e:
            self.get_logger().error(f'Falta una clave en el diccionario: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = TestSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()