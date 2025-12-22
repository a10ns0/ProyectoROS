import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import numpy as np

class MonitorDual(Node):
    def __init__(self):
        super().__init__('monitor_dual')
        
        # --- OÍDO 1: SENSOR PERFIL (ROSADO) ---
        self.sub_perfil = self.create_subscription(
            LaserScan,
            '/scan_perfil',    # Tópico esperado para el perfil
            self.callback_perfil,
            10)
        
        # --- OÍDO 2: SENSOR LONGITUDINAL (AMARILLO) ---
        self.sub_longitudinal = self.create_subscription(
            LaserScan,
            '/scan_longitudinal', # Tópico esperado para el lateral
            self.callback_longitudinal,
            10)
        
        # Datos
        self.x_pink, self.y_pink = [], []
        self.x_yellow, self.y_yellow = [], []

        # --- CONFIGURACIÓN GRÁFICA (Estilo Industrial) ---
        plt.ion()
        # Creamos una figura con 2 paneles (arriba y abajo)
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(8, 10))
        self.fig.canvas.manager.set_window_title('SICK :: Sistema de Posicionamiento Dual')
        self.fig.patch.set_facecolor('black')

        # === PANEL 1: VISTA TRASERA (PERFIL - ROSA) ===
        self.setup_axis(self.ax1, "VISTA PERFIL (Trasera)", 2.0, 3.0)
        self.line_pink, = self.ax1.plot([], [], '.', color='magenta', markersize=3)
        # Linea de referencia (Suelo/Techo)
        self.ax1.axhline(y=2.0, color='red', linestyle='--', linewidth=0.5)

        # === PANEL 2: VISTA SUPERIOR (LONGITUDINAL - AMARILLA) ===
        self.setup_axis(self.ax2, "VISTA SUPERIOR (Alineación)", 5.0, 8.0)
        self.line_yellow, = self.ax2.plot([], [], '.', color='yellow', markersize=3)

    def setup_axis(self, ax, title, limit_x, limit_y):
        """Configura el estilo oscuro para cada panel"""
        ax.set_facecolor('black')
        ax.set_title(title, color='white', fontsize=10)
        ax.spines['bottom'].set_color('white')
        ax.spines['left'].set_color('white')
        ax.tick_params(axis='x', colors='white')
        ax.tick_params(axis='y', colors='white')
        ax.grid(True, color='#333333', linestyle='--')
        ax.set_xlim(-limit_x, limit_x)
        ax.set_ylim(0, limit_y)

    def callback_perfil(self, msg):
        # Procesamiento para la línea ROSADA (Tu lógica anterior)
        # Filtros: Max dist 2.0m, Angulo recortado a 30 grados
        ranges, angles = self.process_scan(msg, 2.0, 30.0, 30.0)
        self.x_pink = ranges * np.sin(angles)
        self.y_pink = ranges * np.cos(angles)

    def callback_longitudinal(self, msg):
        # Procesamiento para la línea AMARILLA
        # Filtros: Max dist 8.0m, Angulo amplio (90 grados)
        ranges, angles = self.process_scan(msg, 8.0, 90.0, 90.0)
        self.x_yellow = ranges * np.sin(angles)
        self.y_yellow = ranges * np.cos(angles)

    def process_scan(self, msg, max_dist, limit_left_deg, limit_right_deg):
        """Función auxiliar matemática para limpiar datos"""
        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        
        mask = np.isfinite(ranges) & (ranges > 0.1) & (ranges < max_dist)
        
        left_rad = np.radians(limit_left_deg)
        right_rad = np.radians(limit_right_deg)
        mask_angle = (angles < left_rad) & (angles > -right_rad)
        
        final_mask = mask & mask_angle
        return ranges[final_mask], angles[final_mask]

    def update_plot(self):
        self.line_pink.set_data(self.x_pink, self.y_pink)
        self.line_yellow.set_data(self.x_yellow, self.y_yellow)
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    monitor = MonitorDual()
    try:
        while rclpy.ok():
            rclpy.spin_once(monitor, timeout_sec=0.1)
            monitor.update_plot()
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()
        plt.close()

if __name__ == '__main__':
    main()
