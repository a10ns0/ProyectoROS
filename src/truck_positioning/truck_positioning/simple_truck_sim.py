import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

class TruckSimulatorHibrido(Node):
    def __init__(self):
        super().__init__('truck_simulator_hibrido')
        
        # Solo necesitamos el TF para mover el camión virtual
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Escuchamos los comandos del HMI (W, S, Space)
        self.create_subscription(String, '/truck_cmd', self.cmd_callback, 10)
        
        # --- ESTADO DEL CAMIÓN ---
        self.truck_x = -15.0 
        self.velocidad = 0.0
        self.dt = 0.05        
        
        # Timer para física de movimiento
        self.timer = self.create_timer(self.dt, self.update_simulation)
        
        self.get_logger().info(">>> MODO HIL ACTIVADO: LÁSERES SIMULADOS APAGADOS <<<")
        self.get_logger().info(">>> Esperando datos de sensores reales SICK/Hokuyo... <<<")

    def cmd_callback(self, msg):
        if msg.data == "FORWARD": self.velocidad = 2.0
        elif msg.data == "BACKWARD": self.velocidad = -2.0
        elif msg.data == "STOP": self.velocidad = 0.0

    def update_simulation(self):
        # 1. Calcular nueva posición
        self.truck_x += self.velocidad * self.dt
        
        # 2. Publicar la posición del camión (Base Link)
        # Esto permite que el HMI dibuje el camión azul/rojo
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.truck_x
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = TruckSimulatorHibrido()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
