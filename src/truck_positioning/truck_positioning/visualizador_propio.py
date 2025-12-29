import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import open3d as o3d
import numpy as np
import threading
import sys
import time

class VisualizadorCamion(Node):
    def __init__(self):
        super().__init__('visualizador_3d_custom')
        
        # ==========================================
        # 1. CONFIGURACIÓN DE SENSORES
        # ==========================================
        self.CFG_S1 = { # Sensor Azul (Estructura)
            'pos': [0.0, 0.0, 1.5],    
            'dist_min': 0.1, 'dist_max': 4.0,
            'ang_min': -90.0, 'ang_max': 0
        }
        self.CFG_S2 = { # Sensor Rojo (Perfil)
            'pos': [2.0, 0.0, 1.5],    
            'dist_min': 0.1, 'dist_max': 1.3,
            'ang_min': -90.0, 'ang_max': 90.0
        }

        # --- TOPICS ---
        self.create_subscription(LaserScan, '/scan_estructura', self.callback_antiguo, 10)
        self.create_subscription(LaserScan, '/scan_distancia', self.callback_long, 10)

        # Buffers de puntos
        self.pcd_antiguo = o3d.geometry.PointCloud()
        self.pcd_long = o3d.geometry.PointCloud()
        self.new_data_antiguo = False
        self.new_data_long = False

        # --- INTERFAZ OPEN3D ---
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name="Sistema Minero - Vista 3D", width=1280, height=720)
        
        # Fondo gris oscuro profesional
        opt = self.vis.get_render_option()
        if opt is not None:
            opt.background_color = np.asarray([0.15, 0.15, 0.15]) 
            opt.point_size = 4.0
            opt.show_coordinate_frame = True # Muestra el eje RGB pequeño en la esquina

        # 1. AGREGAR SUELO (GRID)
        self.grid = self.crear_grilla(size=40, step=2) # 40x40 metros, lineas cada 2m
        self.vis.add_geometry(self.grid)

        # 2. AGREGAR MARCADORES DE SENSORES
        # Sensor 1 (Azul)
        s1 = o3d.geometry.TriangleMesh.create_sphere(radius=0.2)
        s1.paint_uniform_color([0, 1, 1])
        s1.translate(self.CFG_S1['pos'])
        self.vis.add_geometry(s1)

        # Sensor 2 (Rojo)
        s2 = o3d.geometry.TriangleMesh.create_sphere(radius=0.2)
        s2.paint_uniform_color([1, 0, 0])
        s2.translate(self.CFG_S2['pos'])
        self.vis.add_geometry(s2)

        # Eje de coordenadas grande en el origen (0,0,0)
        axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])
        self.vis.add_geometry(axis)

        # Nubes de puntos
        self.vis.add_geometry(self.pcd_antiguo)
        self.vis.add_geometry(self.pcd_long)

        # 3. CONFIGURAR CÁMARA INICIAL
        self.configurar_camara()

        self.running = True

    def crear_grilla(self, size=40, step=2):
        """Dibuja una rejilla tipo Rviz en el suelo (Z=0)"""
        lines = []
        points = []
        
        # Líneas paralelas al eje X y al eje Y
        # El rango va desde -size/2 hasta +size/2
        start = -size / 2
        end = size / 2
        num_lines = int(size / step) + 1

        # Generar vértices
        for i in range(num_lines):
            coord = start + i * step
            
            # Línea paralela a X
            points.append([start, coord, 0])
            points.append([end, coord, 0])
            lines.append([len(points)-2, len(points)-1])
            
            # Línea paralela a Y
            points.append([coord, start, 0])
            points.append([coord, end, 0])
            lines.append([len(points)-2, len(points)-1])

        colors = [[0.3, 0.3, 0.3] for i in range(len(lines))] # Color gris tenue
        
        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(points)
        line_set.lines = o3d.utility.Vector2iVector(lines)
        line_set.colors = o3d.utility.Vector3dVector(colors)
        
        return line_set

    def configurar_camara(self):
        """Fuerza la cámara a mirar al origen desde una perspectiva isométrica"""
        ctr = self.vis.get_view_control()
        
        # Parámetros de cámara:
        # lookat: A dónde mira la cámara (El centro del camión/suelo)
        # front: Desde dónde mira (Vector dirección). (-1, -1, 1) es diagonal arriba-atrás.
        # up: Qué eje es "arriba" (Z)
        # zoom: Qué tan cerca está (menor número = más cerca)
        
        ctr.set_lookat([2.0, 0.0, 0.0]) # Mirar un poco adelante del origen
        ctr.set_front([-0.5, -0.5, 0.5]) # Vista diagonal
        ctr.set_up([0.0, 0.0, 1.0])      # Z es arriba
        ctr.set_zoom(0.4)                # Zoom general

    def laser_to_xyz(self, msg, x_offset, y_offset, z_offset, yaw_rot, pitch_rot, roll_rot, config):
        # 1. Extraer y recortar datos
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        ranges = np.array(msg.ranges)
        min_len = min(len(angles), len(ranges))
        angles = angles[:min_len]
        ranges = ranges[:min_len]

        # 2. Filtro de Configuración
        lim_ang_min_rad = np.radians(config['ang_min'])
        lim_ang_max_rad = np.radians(config['ang_max'])
        
        valid = (ranges > 0.1) & \
                (ranges > config['dist_min']) & (ranges < config['dist_max']) & \
                (angles > lim_ang_min_rad) & (angles < lim_ang_max_rad)

        r = ranges[valid]
        a = angles[valid]
        
        if len(r) == 0: return np.zeros((0, 3))
        
        # 3. Coordenadas Base (El sensor escanea en su propio plano X-Y)
        # Por defecto: X=Adelante, Y=Izquierda, Z=0
        x = r * np.cos(a)
        y = r * np.sin(a)
        z = np.zeros_like(x)

        # 4. ROTACIÓN 1: ROLL (Giro sobre eje X)
        # Útil para poner el sensor de lado (Vertical)
        if roll_rot != 0:
            c, s = np.cos(roll_rot), np.sin(roll_rot)
            # y' = y*cos - z*sin
            # z' = y*sin + z*cos
            y_new = y * c - z * s
            z_new = y * s + z * c
            y = y_new
            z = z_new

        # 5. ROTACIÓN 2: PITCH (Giro sobre eje Y)
        # Útil para mirar arriba/abajo
        if pitch_rot != 0:
            c, s = np.cos(pitch_rot), np.sin(pitch_rot)
            # x' = x*cos + z*sin (Ojo con el signo según convención)
            # z' = -x*sin + z*cos
            x_new = x * c - z * s
            z_new = x * s + z * c
            x = x_new
            z = z_new

        # 6. ROTACIÓN 3: YAW (Giro sobre eje Z)
        # Útil para mirar izquierda/derecha
        if yaw_rot != 0:
            c, s = np.cos(yaw_rot), np.sin(yaw_rot)
            # x' = x*cos - y*sin
            # y' = x*sin + y*cos
            x_new = x * c - y * s
            y_new = x * s + y * c
            x = x_new
            y = y_new

        # 7. TRASLACIÓN FINAL
        return np.vstack((x + x_offset, y + y_offset, z + z_offset)).T


    def callback_antiguo(self, msg):
        try:
            points = self.laser_to_xyz(msg, 
                self.CFG_S1['pos'][0], self.CFG_S1['pos'][1], self.CFG_S1['pos'][2], 
                yaw_rot=np.radians(0),   
                pitch_rot=np.radians(0),  
                roll_rot=np.radians(90),    # <--- NUEVO: Roll en 0 (plano)
                config=self.CFG_S1)
            
            self.pcd_antiguo.points = o3d.utility.Vector3dVector(points)
            self.pcd_antiguo.paint_uniform_color([0, 1, 1]) 
            self.new_data_antiguo = True
        except: pass

    def callback_long(self, msg):
        try:
            points = self.laser_to_xyz(msg, 
                self.CFG_S2['pos'][0], self.CFG_S2['pos'][1], self.CFG_S2['pos'][2], 
                yaw_rot=np.radians(0), 
                pitch_rot=np.radians(-90),
                roll_rot=np.radians(0),   # <--- EQUIVALENTE AL ANTIGUO "True"
                config=self.CFG_S2)
            
            self.pcd_long.points = o3d.utility.Vector3dVector(points)
            self.pcd_long.paint_uniform_color([1, 0, 0]) 
            self.new_data_long = True
        except: pass
def main(args=None):
    rclpy.init(args=args)
    gui = VisualizadorCamion()
    ros_thread = threading.Thread(target=rclpy.spin, args=(gui,), daemon=True)
    ros_thread.start()

    try:
        while gui.running:
            if gui.new_data_antiguo:
                gui.vis.update_geometry(gui.pcd_antiguo)
                gui.new_data_antiguo = False
            if gui.new_data_long:
                gui.vis.update_geometry(gui.pcd_long)
                gui.new_data_long = False

            gui.vis.poll_events()
            gui.vis.update_renderer()
            time.sleep(0.01) 
            
    except KeyboardInterrupt: pass
    finally:
        gui.vis.destroy_window()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
