import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy # <--- ¡ESTO ES NUEVO!

class LaserReader(Node):
    def __init__(self):
        super().__init__('lector_laser')
        
        # --- CONFIGURACIÓN DE COMPATIBILIDAD (QoS) ---
        # Esto permite escuchar al sensor aunque envíe datos en modo "Best Effort"
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        # ---------------------------------------------

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            qos_profile) # <--- Usamos el perfil compatible aquí
            
        self.get_logger().info('--- Lector de Laser Iniciado (Modo Compatible) ---')

    def listener_callback(self, msg):
        # Indice central = lo que tiene justo enfrente
        indice_central = len(msg.ranges) // 2
        distancia = msg.ranges[indice_central]

        if distancia < 0.1:
            estado = "Muy Cerca / Error"
        elif distancia > 40.0:
            estado = "Fuera de rango"
        else:
            estado = f"{distancia:.2f} metros"

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