import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

# Implementación simple de Filtro de Kalman 1D
class KalmanFilter1D:
    def __init__(self, dt, u_noise, m_noise):
        self.dt = dt
        self.x = 0.0  # Posición
        self.v = 0.0  # Velocidad
        self.P = [[100.0, 0.0], [0.0, 100.0]] 
        self.Q = [[u_noise, 0.0], [0.0, u_noise]] 
        self.R = m_noise

    def predict(self):
        self.x = self.x + self.v * self.dt
        self.P[0][0] += self.dt * self.P[1][1] * self.dt + self.Q[0][0]
        self.P[1][1] += self.Q[1][1]

    def update(self, z):
        S = self.P[0][0] + self.R
        K0 = self.P[0][0] / S
        K1 = self.P[1][0] / S
        y = z - self.x
        self.x += K0 * y
        self.v += K1 * y
        self.P[0][0] -= K0 * S * K0
        self.P[1][1] -= K1 * S * K1
        return self.x

class TruckControlNode(Node):
    def __init__(self):
        super().__init__('truck_control_node')
        
        self.sub_pos = self.create_subscription(Float32, '/tps/truck_tail_position', self.callback_pos, 10)
        self.pub_light = self.create_publisher(String, '/tps/traffic_light', 10)
        
        # Filtro de Kalman (20Hz -> dt=0.05)
        self.kf = KalmanFilter1D(dt=0.05, u_noise=0.1, m_noise=0.2)
        
        # --- CONFIGURACIÓN DE OPERACIÓN ---
        # CAMBIAR ESTO SEGÚN EL CASO (O recibirlo por otro topic)
        self.CONTAINER_TYPE = "40FT"  # Opciones: "20FT", "40FT"
        
        # Setpoints calculados (Desde inicio de zona 0.0)
        self.SETPOINT_40FT = 2.54  
        self.SETPOINT_20FT = 0.00  
        
        self.get_logger().info(f'Control TPS Iniciado. Modo: {self.CONTAINER_TYPE}')

    def callback_pos(self, msg):
        raw_x = msg.data
        
        # 1. Filtrado
        self.kf.predict()
        filtered_x = self.kf.update(raw_x)
        velocity = self.kf.v
        
        # 2. Selección de Objetivo
        target = self.SETPOINT_40FT if self.CONTAINER_TYPE == "40FT" else self.SETPOINT_20FT
        
        # 3. Cálculo de Distancia al Objetivo
        # Distancia positiva = Falta avanzar. Negativa = Se pasó.
        distance_remaining = target - filtered_x
        
        # 4. Lógica de Semáforo
        light_status = ""
        
        if distance_remaining < -0.30:
            light_status = "ROJO PARPADEANTE (RETROCEDA)"
        elif -0.10 <= distance_remaining <= 0.10:
            # Tolerancia de +/- 10cm para el OK
            light_status = "ROJO (STOP - OK)"
        elif distance_remaining <= 2.0:
            light_status = "AMARILLO (DESPACIO)"
        else:
            light_status = "VERDE (AVANCE)"
            
        # Publicar
        msg_str = String()
        msg_str.data = f"[{self.CONTAINER_TYPE}] D:{distance_remaining:.2f}m | V:{velocity:.2f}m/s | {light_status}"
        self.pub_light.publish(msg_str)

def main(args=None):
    rclpy.init(args=args)
    node = TruckControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()