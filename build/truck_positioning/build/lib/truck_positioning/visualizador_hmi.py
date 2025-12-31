import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer
from tf2_ros.transform_listener import TransformListener
import open3d as o3d
import numpy as np
import threading
import time

class VisualizadorSTS_HMI(Node):
    def __init__(self):
        super().__init__('visualizador_sts_hmi')
        
        # --- CONFIGURACIÓN DE SENSORES (LA TUYA EXACTA) ---
        self.CFG_LONG = {
            'pos': [10.2, 0.0, 12.5],
            'dist_min': 0.1, 'dist_max': 20,
            'ang_min': -95.0, 'ang_max': 0,
            'pitch': np.radians(-90), 'yaw': np.radians(0), 'roll': np.radians(90)
        }

        self.CFG_ESTRUC = {
            'pos': [-10.2, 11.19 , 12.5],
            'dist_min': 0.1, 'dist_max': 70,
            'ang_min': -55.0, 'ang_max': 55.0,
            'pitch': np.radians(-90), 'yaw': np.radians(0), 'roll': np.radians(0)
        }

        # --- ZONAS Y LÓGICA ---
        self.DB_TARGETS = {
            '40': {'min': [-6.0, -1.2, 0.0], 'max': [6.0, 1.2, 5.0]},
            '20': {'min': [-3.0, -1.2, 0.0], 'max': [3.0, 1.2, 5.0]}
        }
        self.ZONA_PERFIL = {'min': [-20.0, -3.0, 0.0], 'max': [-5.0, 3.0, 5.0]} # Zona amplia para captar estructura
        
        self.modo_box = '40'
        self.target_box = self.DB_TARGETS['40']
        
        self.perfil_ok = False
        self.posicion_x = None
        self.timer_exito = None

        # --- ROS SETUP ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.create_subscription(LaserScan, '/scan_distancia', self.cb_long, 10)
        self.create_subscription(LaserScan, '/scan_estructura', self.cb_estruc, 10)
        self.pub_cmd = self.create_publisher(String, '/truck_cmd', 10)

        # Buffer Grafico
        self.pts_long = np.zeros((0,3))
        self.pts_estruc = np.zeros((0,3))
        self.truck_last_x = -15.0
        self.actualizar = False

        self.init_gui()
        self.running = True

    def init_gui(self):
        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.vis.create_window("STS HMI FINAL", 1280, 720)
        self.vis.get_render_option().background_color = [0,0,0] # Negro
        self.vis.get_render_option().point_size = 4.0

        # Controles
        self.vis.register_key_callback(ord("W"), lambda v: self.pub_cmd.publish(String(data="FORWARD")))
        self.vis.register_key_callback(ord("S"), lambda v: self.pub_cmd.publish(String(data="BACKWARD")))
        self.vis.register_key_callback(ord(" "), lambda v: self.pub_cmd.publish(String(data="STOP")))
        
        # Geometrías
        self.bbox_target = o3d.geometry.AxisAlignedBoundingBox(
            np.array(self.target_box['min']), np.array(self.target_box['max']))
        self.bbox_target.color = [1, 0, 0]

        self.pcd_long = o3d.geometry.PointCloud()
        self.pcd_estruc = o3d.geometry.PointCloud()
        
        self.mesh_truck = self.crear_camion()
        self.grilla = self.crear_grilla()
        
        # Esferas sensores (Para verificar visualmente la posición)
        s1 = o3d.geometry.TriangleMesh.create_sphere(0.3); s1.translate(self.CFG_ESTRUC['pos']); s1.paint_uniform_color([0,1,1])
        s2 = o3d.geometry.TriangleMesh.create_sphere(0.3); s2.translate(self.CFG_LONG['pos']); s2.paint_uniform_color([1,0.5,0])

        self.vis.add_geometry(self.bbox_target)
        self.vis.add_geometry(self.pcd_long)
        self.vis.add_geometry(self.pcd_estruc)
        self.vis.add_geometry(self.mesh_truck)
        self.vis.add_geometry(self.grilla)
        self.vis.add_geometry(s1)
        self.vis.add_geometry(s2)
        
        ctr = self.vis.get_view_control()
        ctr.set_lookat([0,0,0]); ctr.set_front([0.0, -0.6, 0.8]); ctr.set_zoom(0.5)

    def cb_long(self, msg):
        # Procesar nube longitudinal
        self.pts_long = self.procesar_scan(msg, self.CFG_LONG)
        
        # Lógica de detección Posición (Promedio en X)
        # Filtramos para que solo tome puntos "altos" (camión) y no piso
        mask = (self.pts_long[:,2] > 0.5)
        pts_validos = self.pts_long[mask]
        
        if len(pts_validos) > 5:
            self.posicion_x = np.mean(pts_validos[:,0])
        else:
            self.posicion_x = None
            
        self.evaluar_semaforo()
        self.actualizar = True

    def cb_estruc(self, msg):
        # Procesar nube estructura
        self.pts_estruc = self.procesar_scan(msg, self.CFG_ESTRUC)
        
        # Lógica de detección Perfil
        # Buscamos puntos en la zona de paso del camión
        # Como el sensor está desplazado, miramos si hay puntos cerca de Y=0
        mask = (self.pts_estruc[:,2] > 0.5) & (abs(self.pts_estruc[:,1]) < 2.0)
        pts_validos = self.pts_estruc[mask]
        
        if len(pts_validos) > 5:
            alto = np.max(pts_validos[:,2])
            ancho = np.max(pts_validos[:,1]) - np.min(pts_validos[:,1])
            if alto > 0.5:
                self.perfil_ok = True
                # print(f"Perfil OK: Alto {alto:.2f}m")
        
        self.evaluar_semaforo()
        self.actualizar = True

    def evaluar_semaforo(self):
        if not self.perfil_ok or self.posicion_x is None:
            self.bbox_target.color = [1,0,0] # Rojo
            self.timer_exito = None
            return
            
        centro_meta = (self.target_box['min'][0] + self.target_box['max'][0]) / 2.0
        error = abs(self.posicion_x - centro_meta)
        
        if error < 0.5: # Tolerancia
            if self.timer_exito is None: self.timer_exito = time.time()
            if (time.time() - self.timer_exito) > 3.0:
                self.bbox_target.color = [0,1,0] # Verde
                print(f"VERDE - Error: {error:.2f}m")
            else:
                self.bbox_target.color = [1,1,0] # Amarillo esperando
        else:
            self.bbox_target.color = [1,1,0] # Amarillo ajustando
            self.timer_exito = None

    def procesar_scan(self, msg, cfg):
        # 1. Polar a Cartesiano local
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        r = np.array(msg.ranges)
        valid = (r > cfg['dist_min']) & (r < cfg['dist_max'])
        r = r[valid]; angles = angles[valid]
        
        if len(r) == 0: return np.zeros((0,3))
        
        # En ROS LaserScan estandar: X es al frente, Y a izquierda.
        x = r * np.cos(angles)
        y = r * np.sin(angles)
        z = np.zeros_like(x)
        points = np.vstack((x, y, z)).T
        
        # 2. Rotación (Roll, Pitch, Yaw)
        # Matriz de rotación R = Rz(yaw) * Ry(pitch) * Rx(roll)
        R = o3d.geometry.get_rotation_matrix_from_xyz((cfg['roll'], cfg['pitch'], cfg['yaw']))
        points = points @ R.T # Aplicar rotación
        
        # 3. Traslación
        points += np.array(cfg['pos'])
        
        return points

    def crear_camion(self):
        chasis = o3d.geometry.TriangleMesh.create_box(13.5, 2.4, 1.2)
        chasis.translate([0, -1.2, 0])
        chasis.paint_uniform_color([0,0,0.8])
        cabina = o3d.geometry.TriangleMesh.create_box(2.5, 2.5, 3.0)
        cabina.translate([10.0, -1.25, 0])
        cabina.paint_uniform_color([0.8,0,0])
        camion = chasis + cabina
        camion.translate([-15.0, 0, 0])
        return camion

    def crear_grilla(self):
        lines = o3d.geometry.LineSet()
        pts = []; cx = []
        for i in range(-20, 21, 2):
            pts.append([i, -10, 0]); pts.append([i, 10, 0])
            pts.append([-20, i/2, 0]); pts.append([20, i/2, 0])
            cx.append([len(pts)-4, len(pts)-3]); cx.append([len(pts)-2, len(pts)-1])
        lines.points = o3d.utility.Vector3dVector(pts)
        lines.lines = o3d.utility.Vector2iVector(cx)
        lines.paint_uniform_color([0.2, 0.2, 0.2])
        return lines

def main():
    rclpy.init()
    gui = VisualizadorSTS_HMI()
    t = threading.Thread(target=rclpy.spin, args=(gui,), daemon=True)
    t.start()
    try:
        while gui.running:
            # Mover camion visual
            try:
                now = rclpy.time.Time()
                if gui.tf_buffer.can_transform('map', 'base_link', now):
                    t = gui.tf_buffer.lookup_transform('map', 'base_link', now)
                    dx = t.transform.translation.x - gui.truck_last_x
                    if abs(dx) > 0.001:
                        gui.mesh_truck.translate([dx, 0, 0])
                        gui.vis.update_geometry(gui.mesh_truck)
                        gui.truck_last_x = t.transform.translation.x
            except: pass
            
            if gui.actualizar:
                gui.pcd_long.points = o3d.utility.Vector3dVector(gui.pts_long)
                gui.pcd_long.paint_uniform_color([1, 0.5, 0])
                gui.pcd_estruc.points = o3d.utility.Vector3dVector(gui.pts_estruc)
                gui.pcd_estruc.paint_uniform_color([0, 1, 1])
                gui.vis.update_geometry(gui.pcd_long)
                gui.vis.update_geometry(gui.pcd_estruc)
                gui.vis.update_geometry(gui.bbox_target)
                gui.actualizar = False
                
            gui.vis.poll_events()
            gui.vis.update_renderer()
            time.sleep(0.01)
    except KeyboardInterrupt: pass
    finally: gui.vis.destroy_window(); rclpy.shutdown()

if __name__ == '__main__': main()
