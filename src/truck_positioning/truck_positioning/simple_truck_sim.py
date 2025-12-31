import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
import numpy as np
import math

class TruckSimulator(Node):
    def __init__(self):
        super().__init__('truck_simulator')
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pub_scan_estruc = self.create_publisher(LaserScan, '/scan_estructura', 10)
        self.pub_scan_long = self.create_publisher(LaserScan, '/scan_distancia', 10)
        self.sub_cmd = self.create_subscription(String, '/truck_cmd', self.cmd_callback, 10)
        
        # --- FÍSICA CAMIÓN ---
        self.truck_x = -15.0 
        self.velocidad = 0.0
        self.dt = 0.05        
        
        # Dimensiones del camión (Hitbox)
        self.L_CHASIS = 13.5
        self.W_CHASIS = 2.4
        self.H_CHASIS = 1.2
        self.CABIN_START = 10.0 # Metros desde la cola
        self.H_CABIN = 3.0
        self.W_CABIN = 2.5
        
        # --- CONFIGURACIÓN DE SENSORES (EXACTA A TU PEDIDO) ---
        # NOTA: Estas coordenadas son fijas en el mundo.
        
        # SENSOR 1: ESTRUCTURA (Desplazado en Y y X negativo)
        self.S1_POS = [-10.2, 11.19, 12.5] 
        
        # SENSOR 2: LONGITUDINAL (Centrado en Y, X positivo)
        self.S2_POS = [10.2, 0.0, 12.5]    
        
        self.timer = self.create_timer(self.dt, self.update_simulation)
        self.get_logger().info(">>> SIMULADOR FISICO INICIADO CON NUEVAS COORDENADAS <<<")

    def cmd_callback(self, msg):
        if msg.data == "FORWARD": self.velocidad = 2.0
        elif msg.data == "BACKWARD": self.velocidad = -2.0
        else: self.velocidad = 0.0

    def get_hit_height(self, x_world, y_world):
        """ Retorna la altura del objeto en una coordenada X,Y del mundo """
        # Convertir a coordenadas locales del camión
        x_local = x_world - self.truck_x
        y_local = y_world # Asumimos camión centrado en Y=0
        
        # Verificar si está dentro del largo del camión
        if 0 <= x_local <= self.L_CHASIS:
            # Verificar zona Cabina
            if x_local >= self.CABIN_START:
                if abs(y_local) <= (self.W_CABIN / 2.0):
                    return self.H_CABIN
            # Verificar zona Chasis
            else:
                if abs(y_local) <= (self.W_CHASIS / 2.0):
                    return self.H_CHASIS
        return 0.0 # Suelo

    def simular_lidar_estructura(self):
        """ 
        Simula el sensor en [-10.2, 11.19, 12.5]. 
        Al tener Pitch -90 y Roll 0, escanea en el plano Y-Z del mundo.
        """
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'sensor_estructura_link'
        
        # Configuración angular (Tuya: -55 a 55)
        msg.angle_min = np.radians(-55.0)
        msg.angle_max = np.radians(55.0)
        msg.range_min = 0.1
        msg.range_max = 70.0
        num_points = 100
        msg.angle_increment = (msg.angle_max - msg.angle_min) / (num_points - 1)
        
        ranges = []
        for i in range(num_points):
            angle = msg.angle_min + i * msg.angle_increment
            
            # Matemática de Raycast:
            # El sensor barre en Y. 
            # dy = Altura * tan(angle). 
            # El ángulo 0 apunta directo abajo (Z-). Ángulo positivo va hacia Y- (izquierda del sensor).
            
            # Proyección al suelo
            dist_suelo_h = self.S1_POS[2] * math.tan(angle)
            
            # Coordenada Y donde el rayo toca el suelo
            # Nota: Dependiendo de la rotación del sensor, ajustamos el signo.
            # Asumimos que angulo positivo barre hacia el centro (Y=0).
            y_target = self.S1_POS[1] - dist_suelo_h 
            x_target = self.S1_POS[0] # En este sensor X es fijo en el barrido
            
            # Chequear colisión
            h_obj = self.get_hit_height(x_target, y_target)
            
            if h_obj > 0:
                # Hipotenusa ajustada a la nueva altura
                cateto_v = self.S1_POS[2] - h_obj
                dist = cateto_v / math.cos(angle)
            else:
                # Distancia al suelo
                dist = self.S1_POS[2] / math.cos(angle)
            
            # Ruido
            dist += np.random.normal(0, 0.02)
            ranges.append(dist)
            
        msg.ranges = ranges
        return msg

    def simular_lidar_longitudinal(self):
        """
        Simula el sensor en [10.2, 0.0, 12.5].
        Con Roll 90, escanea en el plano X-Z del mundo.
        """
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'sensor_longitudinal_link'
        
        # Configuración angular (Tuya: -95 a 0)
        msg.angle_min = np.radians(-95.0)
        msg.angle_max = np.radians(0.0)
        msg.range_min = 0.1
        msg.range_max = 20.0
        num_points = 100
        msg.angle_increment = (msg.angle_max - msg.angle_min) / (num_points - 1)

        ranges = []
        for i in range(num_points):
            angle = msg.angle_min + i * msg.angle_increment
            
            # Raycast en X (Longitudinal)
            # Como angle va de -95 a 0, tan(angle) es negativo (hacia atrás)
            dx = self.S2_POS[2] * math.tan(angle)
            
            x_target = self.S2_POS[0] + dx # + porque tan es negativo y miramos hacia atras
            y_target = self.S2_POS[1]      # Fijo en 0
            
            # Evitar disparar al cielo (Angulos > -pi/2 muy cerca)
            if angle < -1.55: 
                ranges.append(0.0)
                continue

            h_obj = self.get_hit_height(x_target, y_target)
            
            if h_obj > 0:
                cateto_v = self.S2_POS[2] - h_obj
                dist = cateto_v / math.cos(angle)
            else:
                dist = self.S2_POS[2] / math.cos(angle)
            
            dist = abs(dist) + np.random.normal(0, 0.02)
            ranges.append(dist)
            
        msg.ranges = ranges
        return msg

    def update_simulation(self):
        # 1. Mover Camión
        self.truck_x += self.velocidad * self.dt
        
        # 2. Publicar TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.truck_x
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)
        
        # 3. Publicar Sensores
        self.pub_scan_estruc.publish(self.simular_lidar_estructura())
        self.pub_scan_long.publish(self.simular_lidar_longitudinal())

def main(args=None):
    rclpy.init(args=args)
    node = TruckSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
