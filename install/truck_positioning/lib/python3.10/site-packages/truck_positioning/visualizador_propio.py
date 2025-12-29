import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import open3d as o3d
import numpy as np
import threading
import sys

class VisualizadorCamion(Node):
    def __init__(self):
        super().__init__('visualizador_3d_custom')
        
        # --- CONFIGURACIÓN DE SENSORES ---
        # Asegúrate de que estos sean los topics reales de tus sensores
        self.create_subscription(LaserScan, '/scan_antiguo', self.callback_antiguo, 10)
        self.create_subscription(LaserScan, '/scan_longitudinal', self.callback_long, 10)

        # Nubes de puntos (Buffers gráficos)
        self.pcd_antiguo = o3d.geometry.PointCloud()
        self.pcd_long = o3d.geometry.PointCloud()
        
        # --- INTERFAZ GRÁFICA OPEN3D ---
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name="Sistema Minero - Vista 3D", width=1024, height=768)
        
        # Opciones visuales
        # --- BLOQUE DE SEGURIDAD ---
        # Intentamos configurar el color, pero si falla el video, no cerramos el programa
        opt = self.vis.get_render_option()
        if opt is not None:
            opt.background_color = np.asarray([0.1, 0.1, 0.1])
            opt.point_size = 4.0
        else:
            print("⚠️ ADVERTENCIA: Modo sin ventana detectado. Revise su configuración gráfica.")
        
        # Referencias visuales (Suelo y Ejes)
        axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=2.0, origin=[0, 0, 0])
        self.vis.add_geometry(axis)
        
        # Agregamos las nubes vacías al inicio para registrarlas en el motor gráfico
        self.vis.add_geometry(self.pcd_antiguo)
        self.vis.add_geometry(self.pcd_long)

        self.running = True

    def laser_to_xyz(self, msg, x_offset, y_offset, z_offset, yaw_rot, roll_rot_90):
        """Convierte datos polares del LIDAR a coordenadas 3D del mundo real"""
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        ranges = np.array(msg.ranges)
        
        # Filtro: Limpiar datos inválidos (infinitos o muy cercanos)
        valid = (ranges > msg.range_min) & (ranges < msg.range_max)
        r = ranges[valid]
        a = angles[valid]
        
        if len(r) == 0: return np.zeros((0, 3)) # Si no hay datos, retornar vacío
        
        # 1. Conversión básica (Polar -> Cartesiano 2D local del sensor)
        x = r * np.cos(a)
        y = r * np.sin(a)
        z = np.zeros_like(x) # El láser es 2D, así que Z es plano localmente
        
        # 2. Transformación al mundo 3D (Rotación y Traslación)
        
        # CASO A: Sensor Vertical (Roll 90°) - El eje X local apunta al suelo/lado
        if roll_rot_90:
            # Rotación: Lo que el sensor ve como X, en el mundo es Z (altura) o Y.
            # Asumiendo montaje estándar de perfil:
            # El plano de escaneo es vertical perpendicular al avance.
            x_final = np.full_like(x, x_offset) # Fijo en la posición longitudinal (5m)
            y_final = x                 # La profundidad del barrido
            z_final = y + z_offset      # La altura
            
            # Nota: Si el barrido sale al revés, invertimos signos aquí.
            return np.vstack((x_final, y_final, z_final)).T
            
        # CASO B: Sensor Diagonal (Yaw 45°)
        if yaw_rot != 0:
            c, s = np.cos(yaw_rot), np.sin(yaw_rot)
            # Rotación en eje Z (Yaw)
            x_rot = x * c - y * s
            y_rot = x * s + y * c
            
            x_final = x_rot + x_offset
            y_final = y_rot + y_offset
            z_final = z + z_offset # Sumamos la altura de montaje (3m)
            return np.vstack((x_final, y_final, z_final)).T

        # CASO C: Sensor plano normal (si hubiera)
        return np.vstack((x + x_offset, y + y_offset, z + z_offset)).T

    def callback_antiguo(self, msg):
        # Sensor 1: Origen (0,0,3), Yaw=45° (0.785 rad)
        points = self.laser_to_xyz(msg, x_offset=0, y_offset=0, z_offset=3.0, yaw_rot=0.785, roll_rot_90=False)
        
        self.pcd_antiguo.points = o3d.utility.Vector3dVector(points)
        self.pcd_antiguo.paint_uniform_color([0, 0.5, 1]) # Azul Claro
        self.update_vis()

    def callback_long(self, msg):
        # Sensor 2: En X=5m, Z=3m, Vertical (Roll 90°)
        points = self.laser_to_xyz(msg, x_offset=5.0, y_offset=0, z_offset=3.0, yaw_rot=0, roll_rot_90=True)
        
        self.pcd_long.points = o3d.utility.Vector3dVector(points)
        self.pcd_long.paint_uniform_color([1, 0, 0]) # Rojo
        self.update_vis()

    def update_vis(self):
        # Le dice a la tarjeta gráfica que los datos cambiaron
        self.vis.update_geometry(self.pcd_antiguo)
        self.vis.update_geometry(self.pcd_long)
        self.vis.poll_events()
        self.vis.update_renderer()

def main(args=None):
    rclpy.init(args=args)
    gui = VisualizadorCamion()
    
    # Hilo paralelo para recibir datos de ROS sin congelar la ventana
    ros_thread = threading.Thread(target=rclpy.spin, args=(gui,), daemon=True)
    ros_thread.start()

    try:
        while gui.running:
            gui.vis.poll_events()
            gui.vis.update_renderer()
    except KeyboardInterrupt:
        pass
    finally:
        gui.vis.destroy_window()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
