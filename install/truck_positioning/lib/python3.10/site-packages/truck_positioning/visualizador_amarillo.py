import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import numpy as np

class VisualizadorAmarillo(Node):
    def __init__(self):
        super().__init__('visualizador_amarillo')
        
        # OJO: Este nodo escucha un TÓPICO DIFERENTE
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan_amarillo',  # <--- Nombre diferenciado
            self.listener_callback,
            10)
        
        # Configuración de Filtros (Ajustados para visión horizontal/larga)
        self.max_dist_limit = 2.0   # 8 metros (más lejos que el de perfil)
        self.limit_left_deg = 15.0  # Visión amplia
        self.limit_right_deg = 45.0 # Visión amplia
        
        self.x_data = []
        self.y_data = []

        # Configuración Gráfica
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.fig.canvas.manager.set_window_title('Sensor Longitudinal (Linea Amarilla)')
        
        # Estilo oscuro
        self.fig.patch.set_facecolor('black')
        self.ax.set_facecolor('black')
        self.ax.spines['bottom'].set_color('white')
        self.ax.spines['left'].set_color('white')
        self.ax.tick_params(axis='x', colors='white')
        self.ax.tick_params(axis='y', colors='white')
        
        # Puntos del láser (AMARILLOS)
        self.scat, = self.ax.plot([], [], '.', color='yellow', markersize=3)
        
        # Zoom visual (Más amplio porque este sensor mira más lejos)
        self.ax.set_xlim(-5, 5) 
        self.ax.set_ylim(0, 10.0) # Hasta 10 metros
        self.ax.grid(True, color='gray', linestyle='--', linewidth=0.3)

    def listener_callback(self, msg):
        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        
        mask_valid = np.isfinite(ranges) & (ranges > 0.1)
        mask_dist = ranges < self.max_dist_limit
        
        # Filtros de ángulo
        left_rad = np.radians(self.limit_left_deg)
        right_rad = np.radians(self.limit_right_deg)
        mask_angle = (angles < left_rad) & (angles > -right_rad)
        
        final_mask = mask_valid & mask_dist & mask_angle
        
        ranges = ranges[final_mask]
        angles = angles[final_mask]
        
        self.x_data = ranges * np.sin(angles) 
        self.y_data = ranges * np.cos(angles)

    def update_plot(self):
        self.scat.set_data(self.x_data, self.y_data)
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    visualizador = VisualizadorAmarillo()
    try:
        while rclpy.ok():
            rclpy.spin_once(visualizador, timeout_sec=0.1)
            visualizador.update_plot()
    except KeyboardInterrupt:
        pass
    finally:
        visualizador.destroy_node()
        rclpy.shutdown()
        plt.close()

if __name__ == '__main__':
    main()
