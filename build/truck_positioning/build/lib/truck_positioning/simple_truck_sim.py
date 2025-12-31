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
        self.create_subscription(String, '/truck_cmd', self.cmd_callback, 10)
        
        # --- ESTADO DEL CAMIÓN ---
        self.truck_x = -15.0 
        self.velocidad = 0.0
        self.dt = 0.05        
        
        # --- CONFIGURACIÓN DE SENSORES (EXACTA AL VISUALIZADOR) ---
        # Si cambias algo aquí, cámbialo en el visualizador también.
        self.S1_POS = [-10.2, 11.19, 12.5] # Estructura
        self.S2_POS = [10.2, 0.0, 12.5]    # Longitudinal
        
        # --- DEFINICIÓN DE "CAJAS SÓLIDAS" (HITBOXES) ---
        # Esto debe coincidir con: o3d.geometry.TriangleMesh.create_box en tu HMI
        # Coordenadas LOCALES relativas al base_link del camión (X=0 es la cola)
        
        # CHASIS (AZUL): create_box(13.5, 2.4, 1.2)
        self.BOX_CHASIS = {
            'x_min': 0.0, 'x_max': 13.5,
            'y_min': -1.2, 'y_max': 1.2, # Centrado en Y
            'z_top': 1.2
        }
        
        # CABINA (ROJA): create_box(2.5, 2.5, 3.0) trasladada a X=10.0
        self.BOX_CABINA = {
            'x_min': 10.0, 'x_max': 12.5,
            'y_min': -1.25, 'y_max': 1.25,
            'z_top': 3.0
        }

        self.timer = self.create_timer(self.dt, self.update_simulation)
        self.get_logger().info(">>> SIMULADOR: HITBOXES SINCRONIZADAS CON 3D <<<")

    def cmd_callback(self, msg):
        if msg.data == "FORWARD": self.velocidad = 2.0
        elif msg.data == "BACKWARD": self.velocidad = -2.0
        elif msg.data == "STOP": self.velocidad = 0.0

    def raycast_vertical(self, x_world, y_world, z_sensor):
        """
        Simula un rayo cayendo verticalmente (o inclinado) sobre el camión.
        Retorna la DISTANCIA REAL de impacto desde el sensor.
        """
        # 1. Transformar X,Y mundial a coordenadas locales del camión
        x_local = x_world - self.truck_x
        y_local = y_world # Asumimos camión centrado en Y=0
        
        altura_impacto = 0.0 # Por defecto: Suelo
        
        # 2. Chequear colisión con CABINA (Prioridad porque es más alta)
        if (self.BOX_CABINA['x_min'] <= x_local <= self.BOX_CABINA['x_max']) and \
           (self.BOX_CABINA['y_min'] <= y_local <= self.BOX_CABINA['y_max']):
            altura_impacto = self.BOX_CABINA['z_top']
            
        # 3. Si no, chequear colisión con CHASIS
        elif (self.BOX_CHASIS['x_min'] <= x_local <= self.BOX_CHASIS['x_max']) and \
             (self.BOX_CHASIS['y_min'] <= y_local <= self.BOX_CHASIS['y_max']):
            altura_impacto = self.BOX_CHASIS['z_top']
            
        return altura_impacto

    def simular_lidar_estructura(self):
        """ SENSOR 1: Simula barrido en Y (Ancho) """
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'sensor_estruc_link'
        msg.angle_min = np.radians(-55.0); msg.angle_max = np.radians(55.0)
        msg.range_min = 0.1; msg.range_max = 70.0
        msg.angle_increment = (msg.angle_max - msg.angle_min) / 99
        msg.ranges = []
        
        sensor_x, sensor_y, sensor_z = self.S1_POS
        
        for i in range(100):
            angle = msg.angle_min + i * msg.angle_increment
            
            # Trigonometría: El láser viaja lateralmente
            # tan(angle) = dy / dz
            dy = sensor_z * math.tan(angle)
            
            # Punto donde el rayo tocaría el suelo
            target_y = sensor_y - dy # Invertimos signo según orientación
            target_x = sensor_x      # En este sensor X no cambia
            
            h_obj = self.raycast_vertical(target_x, target_y, sensor_z)
            
            # Cálculo de Hipotenusa exacta al punto de impacto
            cateto_z = sensor_z - h_obj
            distancia = cateto_z / math.cos(angle)
            msg.ranges.append(abs(distancia) + np.random.normal(0, 0.01))
            
        return msg

    def simular_lidar_longitudinal(self):
        """ SENSOR 2: Simula barrido en X (Largo) - EL AMARILLO """
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'sensor_long_link'
        msg.angle_min = np.radians(-95.0); msg.angle_max = np.radians(0.0)
        msg.range_min = 0.1; msg.range_max = 40.0
        msg.angle_increment = (msg.angle_max - msg.angle_min) / 99
        msg.ranges = []
        
        sensor_x, sensor_y, sensor_z = self.S2_POS
        
        for i in range(100):
            angle = msg.angle_min + i * msg.angle_increment
            
            # PROTECCIÓN: Si el ángulo apunta hacia arriba o horizonte infinito
            if angle > -0.05 or angle < -1.6: 
                msg.ranges.append(0.0)
                continue

            # Trigonometría Longitudinal: tan(angle) = dx / dz
            # Como angle es negativo (-90 a 0), tan es negativa (hacia atrás)
            dx = sensor_z * math.tan(angle)
            
            # Punto donde tocaría el suelo
            target_x = sensor_x + dx # dx ya es negativo
            target_y = sensor_y      # Centrado
            
            h_obj = self.raycast_vertical(target_x, target_y, sensor_z)
            
            # LA MAGIA: Calcular distancia REAL al objeto, no al suelo
            cateto_z = sensor_z - h_obj
            
            # Si h_obj > 0, el rayo choca antes. Recalculamos.
            distancia = cateto_z / math.cos(angle)
            
            msg.ranges.append(abs(distancia) + np.random.normal(0, 0.01))
            
        return msg

    def update_simulation(self):
        self.truck_x += self.velocidad * self.dt
        
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.truck_x
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)
        
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
