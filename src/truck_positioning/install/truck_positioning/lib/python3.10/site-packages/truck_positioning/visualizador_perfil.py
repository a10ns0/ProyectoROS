import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import numpy as np

class VisualizadorPerfil(Node):
    def __init__(self):
        super().__init__('visualizador_perfil')
        
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan', 
            self.listener_callback,
            10)
        
        # ---------------------------------------------------------
        #  CONFIGURACIÓN DE FILTROS INDEPENDIENTES
        # ---------------------------------------------------------
        
        # 1. Distancia Máxima (El "Techo" o límite de profundidad)
        self.max_dist_limit = 2.0  # Metros

        # 2. Ángulos INDEPENDIENTES (En Grados)
        # Imagina el sensor mirando al frente (0 grados).
        
        self.limit_left_deg = 15.0   # Grados hacia la IZQUIERDA (Lado positivo)
        self.limit_right_deg = 45.0  # Grados hacia la DERECHA   (Lado negativo)

        # Ejemplo: Si pones Left=10 y Right=45, el sensor verá mucho más 
        # hacia la derecha que hacia la izquierda.
        # ---------------------------------------------------------
        
        self.x_data = []
        self.y_data = []

        # Configuración Gráfica
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.fig.canvas.manager.set_window_title('Monitor SICK - Angulos Asimétricos')
        
        # Estilo oscuro
        self.fig.patch.set_facecolor('black')
        self.ax.set_facecolor('black')
        self.ax.spines['bottom'].set_color('white')
        self.ax.spines['left'].set_color('white')
        self.ax.tick_params(axis='x', colors='white')
        self.ax.tick_params(axis='y', colors='white')
        
        # Puntos del láser
        self.scat, = self.ax.plot([], [], '.', color='magenta', markersize=2)
        
        # Linea roja de limite de distancia
        self.ax.axhline(y=self.max_dist_limit, color='red', linestyle='--', linewidth=1, label='Limite')

        # Límites visuales (Zoom fijo para que no baile la pantalla)
        self.ax.set_xlim(-2, 2) 
        self.ax.set_ylim(0, 3.0)
        self.ax.grid(True, color='gray', linestyle='--', linewidth=0.3)

    def listener_callback(self, msg):
        ranges = np.array(msg.ranges)
        # Calcular ángulos para cada punto
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        
        # --- FILTROS ---
        
        # 1. Filtro de Errores
        mask_valid = np.isfinite(ranges) & (ranges > 0.1)
        
        # 2. Filtro de DISTANCIA
        mask_dist = ranges < self.max_dist_limit
        
        # 3. Filtro de ANGULO ASIMÉTRICO
        # Convertimos los grados que pusiste arriba a radianes
        left_rad = np.radians(self.limit_left_deg)
        right_rad = np.radians(self.limit_right_deg)
        
        # La lógica es: El ángulo debe ser MENOR que el límite izquierdo
        # Y MAYOR que el límite derecho (negativo).
        # (Recordando que derecha es negativo en el sistema de coordenadas)
        mask_angle = (angles < left_rad) & (angles > -right_rad)
        
        # Aplicar todos los filtros
        final_mask = mask_valid & mask_dist & mask_angle
        
        ranges_filtered = ranges[final_mask]
        angles_filtered = angles[final_mask]
        
        # Conversión a Cartesianas
        self.x_data = ranges_filtered * np.sin(angles_filtered) 
        self.y_data = ranges_filtered * np.cos(angles_filtered)

    def update_plot(self):
        self.scat.set_data(self.x_data, self.y_data)
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    visualizador = VisualizadorPerfil()
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
