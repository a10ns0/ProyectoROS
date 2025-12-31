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
        
        # ==========================================
        # 1. CONFIGURACIÓN DE SENSORES (TUYA)
        # ==========================================
        self.CFG_LONG = {
            'pos': [10.2, 0.0, 12.5],
            'dist_min': 0.1, 'dist_max': 40, # Aumenté rango maximo
            'ang_min': -95.0, 'ang_max': 0,
            # Roll 90 para barrer a lo largo
            'pitch': np.radians(0), 'yaw': np.radians(-90), 'roll': np.radians(90)
        }

        self.CFG_ESTRUC = {
            'pos': [-10.2, 11.19 , 12.5],
            'dist_min': 0.1, 'dist_max': 70,
            'ang_min': -55.0, 'ang_max': 55.0,
            # Roll 0 para barrer a lo ancho
            'pitch': np.radians(-90), 'yaw': np.radians(180), 'roll': np.radians(0)
        }

        # ==========================================
        # 2. ZONAS Y LÓGICA
        # ==========================================
        self.DB_TARGETS = {
            '40': {'min': [-6.0, -1.2, 0.0], 'max': [6.0, 1.2, 5.0]},
            '20': {'min': [-3.0, -1.2, 0.0], 'max': [3.0, 1.2, 5.0]}
        }
        
        self.modo_box = '40'
        self.target_box = self.DB_TARGETS['40']
        
        # Variables de Estado
        self.perfil_ok = False
        self.posicion_x = None
        self.timer_exito = None
        self.ultimo_debug = 0

        # ==========================================
        # 3. ROS SETUP
        # ==========================================
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.create_subscription(LaserScan, '/scan_distancia', self.cb_long, 10)
        self.create_subscription(LaserScan, '/scan_estructura', self.cb_estruc, 10)
        self.pub_cmd = self.create_publisher(String, '/truck_cmd', 10)

        # Buffers Visuales
        self.pts_long = np.zeros((0,3))
        self.pts_estruc = np.zeros((0,3))
        self.truck_last_x = -15.0
        self.actualizar = False

        self.init_gui()
        self.running = True

    def init_gui(self):
        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.vis.create_window("STS HMI - SEMAFORO", 1280, 720)
        self.vis.get_render_option().background_color = [0,0,0] 
        self.vis.get_render_option().point_size = 4.0

        # Controles
        self.vis.register_key_callback(ord("W"), lambda v: self.pub_cmd.publish(String(data="FORWARD")))
        self.vis.register_key_callback(ord("S"), lambda v: self.pub_cmd.publish(String(data="BACKWARD")))
        self.vis.register_key_callback(ord(" "), lambda v: self.pub_cmd.publish(String(data="STOP")))
        # Reset manual de lógica
        self.vis.register_key_callback(ord("R"), self.reset_logica) 
        self.vis.register_key_callback(ord("1"), lambda v: self.cambiar_caja('20'))
        self.vis.register_key_callback(ord("2"), lambda v: self.cambiar_caja('40'))

        # Geometrías
        self.bbox_target = o3d.geometry.AxisAlignedBoundingBox(
            np.array(self.target_box['min']), np.array(self.target_box['max']))
        self.bbox_target.color = [1, 0, 0] # Rojo Inicial

        self.pcd_long = o3d.geometry.PointCloud()
        self.pcd_estruc = o3d.geometry.PointCloud()
        self.mesh_truck = self.crear_camion()
        self.grilla = self.crear_grilla()
        
        # Esferas de sensores (Referencia visual)
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

    # ==========================================
    # LÓGICA DE SENSORES MEJORADA
    # ==========================================

    def cb_estruc(self, msg):
        """ SENSOR 1: PERFIL (Detecta si pasa un camión) """
        self.pts_estruc = self.procesar_scan(msg, self.CFG_ESTRUC)
        
        # Filtro muy permisivo: Cualquier punto más alto que 0.5m
        # y que esté en el carril central (Ancho +/- 6 metros)
        mask = (self.pts_estruc[:,2] > 0.5) & (abs(self.pts_estruc[:,1]) < 6.0)
        pts_validos = self.pts_estruc[mask]
        
        if len(pts_validos) > 3:
            # Si veo algo alto, activo la memoria (Latch)
            if not self.perfil_ok:
                print(">>> [SENSOR 1] PERFIL VALIDADO: CAMIÓN DETECTADO <<<")
                self.perfil_ok = True
        
        self.evaluar_semaforo()
        self.actualizar = True

    def cb_long(self, msg):
        """ SENSOR 2: POSICIÓN (Mide distancia a la meta) """
        self.pts_long = self.procesar_scan(msg, self.CFG_LONG)
        
        # Filtro permisivo: Cualquier punto alto (camión) en todo el largo del carril
        mask = (self.pts_long[:,2] > 0.5) & (self.pts_long[:,0] > -30) & (self.pts_long[:,0] < 30)
        pts_validos = self.pts_long[mask]
        
        if len(pts_validos) > 3:
            # Calculamos dónde está el frente/centro del camión
            self.posicion_x = np.mean(pts_validos[:,0])
        else:
            self.posicion_x = None
            
        self.evaluar_semaforo()
        self.actualizar = True

    def evaluar_semaforo(self):
        """ LÓGICA DEL CAMBIO DE COLOR """
        
        # DEBUG CADA 1 SEGUNDO PARA NO SATURAR CONSOLA
        if time.time() - self.ultimo_debug > 1.0:
            p_str = f"{self.posicion_x:.2f}" if self.posicion_x else "None"
            print(f"Estado -> Perfil OK: {self.perfil_ok} | Pos X: {p_str}")
            self.ultimo_debug = time.time()

        # 1. ESTADO ROJO:
        # Si el perfil no se ha validado AUN, o el sensor 2 no ve nada.
        if not self.perfil_ok or self.posicion_x is None:
            self.bbox_target.color = [1, 0, 0] # Rojo
            self.timer_exito = None
            return

        # 2. CALCULO DE ERROR (Distancia al centro de la caja)
        centro_meta = (self.target_box['min'][0] + self.target_box['max'][0]) / 2.0
        error = abs(self.posicion_x - centro_meta)
        
        # Tolerancia (0.5 metros)
        ESTA_EN_RANGO = (error <= 0.5)

        if ESTA_EN_RANGO:
            # 3. ESTADO VERDE (O esperando para verde)
            if self.timer_exito is None:
                self.timer_exito = time.time()
                print(">>> EN POSICION... ESPERANDO 3s <<<")
            
            # Chequear tiempo
            if (time.time() - self.timer_exito) > 3.0:
                self.bbox_target.color = [0, 1, 0] # VERDE FIJO
                # print(">>> LUZ VERDE <<<")
            else:
                self.bbox_target.color = [0.5, 1, 0] # Verde Lima (Esperando)
        else:
            # 4. ESTADO AMARILLO (Detectado pero lejos)
            self.bbox_target.color = [1, 1, 0] # Amarillo
            self.timer_exito = None

    # ==========================================
    # UTILIDADES
    # ==========================================
    def procesar_scan(self, msg, cfg):
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        r = np.array(msg.ranges)
        valid = (r > cfg['dist_min']) & (r < cfg['dist_max'])
        r = r[valid]; angles = angles[valid]
        if len(r) == 0: return np.zeros((0,3))
        
        x = r * np.cos(angles); y = r * np.sin(angles); z = np.zeros_like(x)
        points = np.vstack((x, y, z)).T
        
        # Rotación
        R = o3d.geometry.get_rotation_matrix_from_xyz((cfg['roll'], cfg['pitch'], cfg['yaw']))
        points = points @ R.T 
        # Traslación
        points += np.array(cfg['pos'])
        return points

    def cambiar_caja(self, mode):
        self.modo_box = mode
        self.target_box = self.DB_TARGETS[mode]
        self.bbox_target.min_bound = np.array(self.target_box['min'])
        self.bbox_target.max_bound = np.array(self.target_box['max'])
        self.reset_logica(None)
        print(f"Caja cambiada a {mode} pies")

    def reset_logica(self, vis):
        self.perfil_ok = False
        self.bbox_target.color = [1,0,0]
        self.timer_exito = None
        print(">>> SISTEMA RESETEADO (ROJO) <<<")

    def crear_camion(self):
        chasis = o3d.geometry.TriangleMesh.create_box(13.5, 2.4, 1.2)
        chasis.translate([0, -1.2, 0]); chasis.paint_uniform_color([0,0,0.8])
        cabina = o3d.geometry.TriangleMesh.create_box(2.5, 2.5, 3.0)
        cabina.translate([10.0, -1.25, 0]); cabina.paint_uniform_color([0.8,0,0])
        camion = chasis + cabina; camion.translate([-15.0, 0, 0])
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
                
            gui.vis.poll_events(); gui.vis.update_renderer(); time.sleep(0.01)
    except KeyboardInterrupt: pass
    finally: gui.vis.destroy_window(); rclpy.shutdown()

if __name__ == '__main__': main()
