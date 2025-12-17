import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LaserReader(Node):
    def __init__(self):
        super().__init__('lector_laser')
        # Nos suscribimos al tema '/scan'
        # QoS 10 significa que guardamos hasta 10 mensajes en cola
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        self.get_logger().info('--- Lector de Laser Iniciado ---')

    def listener_callback(self, msg):
        # El sensor mide 190 grados.
        # El indice central del array 'ranges' es lo que tiene justo enfrente.
        indice_central = len(msg.ranges) // 2
        distancia = msg.ranges[indice_central]

        # Filtramos errores (a veces da 0.0 o infinito si no ve nada)
        if distancia < 0.1:
            estado = "Muy Cerca / Error"
        elif distancia > 40.0:
            estado = "Fuera de rango"
        else:
            # Mostramos solo 2 decimales
            estado = f"{distancia:.2f} metros"

        # Imprimimos en pantalla
        print(f"Distancia al frente: {estado}")

def main(args=None):
    rclpy.init(args=args)
    nodo = LaserReader()
    
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
