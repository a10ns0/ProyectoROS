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

class VisualizadorFusionado(Node):
    def __init__(self):
        super().__init__('visualizador_sts_fusionado')
        
        # =========================================================
        # 1. CONFIGURACIÓN DE ZONAS Y GEOMETRÍA
        # =========================================================
        self.DB_GEOMETRIA = {
            '40': { 'min': [-6.0, -1.2, 1.0], 'max': [ 6.0,  1.2, 4.5], 'desc': "40 PIES" },
            '20': { 'min': [-3.0, -1.2, 1.0], 'max': [ 3.0,  1.2, 4.5], 'desc': "20 PIES" }
        }
        self.ZONA_PERFIL = { 'min': [-11, -2.5, 0.0], 'max': [ -9.0,  2.5, 5.0] }

        self.modo_actual = '40'
        self.zona_target = self.DB_GEOMETRIA['40']
        
        # Variables de Lógica
        self.perfil_ok = False
        self.posicion_x_camion = None 
        self.ultimo_debug = 0

        # =========================================================
        # 2. CONFIGURACIÓN DE SENSORES (REALES)
        # =========================================================
        self.CFG_LONG = {
            'pos': [10.2, 0.0, 12.5], 
            'dist_min': 0.1, 'dist_max': 40,
            'ang_min': -95.0, 'ang_max': 0, 
            'pitch': np.radians(0), 'yaw': np.radians(180), 'roll': np.radians(90) 
        }

        self.CFG_ESTRUC = {
            'pos': [-10.2, 11.19 , 12.5],
            'dist_min': 0.1, 'dist_max': 70,
            'ang_min': -45.0, 'ang_max': 45.0, # <--- ANGULO CORREGIDO AQUI
            'pitch': np.radians(-90), 'yaw': np.radians(180), 'roll': np.radians(0)
        }

        # =========================================================
        # 3. ESTADO DEL CAMIÓN VIRTUAL (PARA ALERTAS)
        # =========================================================
        self.truck_x_actual = -15.0 # Posición inicial
        # Banderas para no spamear la terminal
        self.flag_s1_detectando = False
        self.flag_s2_detectando = False

        # =========================================================
        # 4. ROS SETUP
        # =========================================================
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.create_subscription(LaserScan, '/scan_distancia', self.cb_longitudinal, 10)
        self.create_subscription(LaserScan, '/scan_estructura', self.cb_estructura, 10)
        self.pub_cmd = self.create_publisher(String, '/truck_cmd', 10)

        # Buffers
        self.puntos_long = np.zeros((0, 3))
        self.puntos_estruc = np.zeros((0, 3))
        self.new_data = False
        self.actualizar_caja = False

        self.init_gui()
        self.running = True

    def init_gui(self):
        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.vis.create_window(window_name="SISTEMA STS - HIL FUSIONADO", width=1280, height=720)
        self.vis.get_render_option().background_color = np.asarray([0.1, 0.1, 0.1])
        self.vis.get_render_option().point_size = 3.0

        # Controles
        self.vis.register_key_callback(ord("1"), lambda v: self.cambiar_modo('20'))
        self.vis.register_key_callback(ord("2"), lambda v: self.cambiar_modo('40'))
        self.vis.register_key_callback(ord("R"), self.reset_logica)
        self.vis.register_key_callback(ord("W"), lambda v: self.pub_cmd.publish(String(data="FORWARD")))
        self.vis.register_key_callback(ord("S"), lambda v: self.pub_cmd.publish(String(data="BACKWARD")))
        self.vis.register_key_callback(ord(" "), lambda v: self.pub_cmd.publish(String(data="STOP")))
        self.vis.register_key_callback(ord("D"), lambda v: None)

        # Geometrías
        self.pcd_long = o3d.geometry.PointCloud()
        self.pcd_estruc = o3d.geometry.PointCloud()
        
        self.box_target = o3d.geometry.AxisAlignedBoundingBox(
            min_bound=np.array(self.zona_target['min']), max_bound=np.array(self.zona_target['max']))
        self.box_target.color = [1, 0, 0]

        self.box_profile = o3d.geometry.AxisAlignedBoundingBox(
            min_bound=np.array(self.ZONA_PERFIL['min']), max_bound=np.array(self.ZONA_PERFIL['max']))
        self.box_profile.color = [0, 0, 1] 

        self.vis.add_geometry(self.box_target)
        self.vis.add_geometry(self.box_profile)
        self.vis.add_geometry(self.crear_suelo()) 
        self.vis.add_geometry(self.pcd_long)
        self.vis.add_geometry(self.pcd_estruc)
        
        # Esferas de sensores
        s1 = o3d.geometry.TriangleMesh.create_sphere(radius=0.3); s1.translate(self.CFG_LONG['pos']); s1.paint_uniform_color([1, 0.5, 0])
        s2 = o3d.geometry.TriangleMesh.create_sphere(radius=0.3); s2.translate(self.CFG_ESTRUC['pos']); s2.paint_uniform_color([0, 1, 1])
        self.vis.add_geometry(s1); self.vis.add_geometry(s2)

        # CAMION 3D
        self.camion_mesh = self.crear_camion_urdf()
        self.vis.add_geometry(self.camion_mesh)

        # Cámara
        ctr = self.vis.get_view_control()
        ctr.set_lookat([0,0,0]); ctr.set_front([0.0, -0.8, 0.8]); ctr.set_up([0,0,1]); ctr.set_zoom(0.5)

    # ==========================================
    # NUEVA FUNCIÓN: ALERTA DE CRUCE VIRTUAL
    # ==========================================
    def chequear_cruce_virtual(self):
        """
        Calcula si el camión virtual está pasando matemáticamente por los sensores.
        El camión mide aprox 12 metros de largo (Desde X-6 a X+6 relativo a su centro).
        """
        camion_min_x = self.truck_x_actual - 6.0
        camion_max_x = self.truck_x_actual + 6.0

        # --- CHEQUEO SENSOR 1 (ESTRUCTURA) en X = -10.2 ---
        pos_s1 = self.CFG_ESTRUC['pos'][0] # -10.2
        # ¿El sensor está 'dentro' del camión?
        en_s1 = (pos_s1 >= camion_min_x) and (pos_s1 <= camion_max_x)

        if en_s1 and not self.flag_s1_detectando:
            print("\n" + "="*50)
            print(">>> [ALERTA] SENSOR 1 (PERFIL) -> CAMIÓN CRUZANDO <<<")
            print(">>> ¡ACTIVA EL SENSOR REAL AHORA! <<<")
            print("="*50 + "\n")
            self.flag_s1_detectando = True
        elif not en_s1 and self.flag_s1_detectando:
            print(">>> [INFO] Camión salió del Sensor 1")
            self.flag_s1_detectando = False

        # --- CHEQUEO SENSOR 2 (LONGITUDINAL) en X = +10.2 ---
        pos_s2 = self.CFG_LONG['pos'][0] # 10.2
        en_s2 = (pos_s2 >= camion_min_x) and (pos_s2 <= camion_max_x)

        if en_s2 and not self.flag_s2_detectando:
            print("\n" + "="*50)
            print(">>> [ALERTA] SENSOR 2 (DISTANCIA) -> CAMIÓN CRUZANDO <<<")
            print(">>> ¡ACTIVA EL SENSOR REAL AHORA! <<<")
            print("="*50 + "\n")
            self.flag_s2_detectando = True
        elif not en_s2 and self.flag_s2_detectando:
            print(">>> [INFO] Camión salió del Sensor 2")
            self.flag_s2_detectando = False

    # ==========================================
    # LÓGICA DE SENSORES (REAL)
    # ==========================================
    def cb_longitudinal(self, msg):
        points = self.laser_to_xyz(msg, self.CFG_LONG)
        self.puntos_long = points
        
        mask = (points[:,1] > -3.0) & (points[:,1] < 3.0) & (points[:,2] > 0.5)
        pts_validos = points[mask]

        if len(pts_validos) > 5:
            self.posicion_x_camion = np.mean(pts_validos[:,0])
        else:
            self.posicion_x_camion = None

        self.evaluar_semaforo()
        self.new_data = True

    def cb_estructura(self, msg):
        points = self.laser_to_xyz(msg, self.CFG_ESTRUC)
        self.puntos_estruc = points

        mask = (points[:,2] > 0.5) & (abs(points[:,1]) < 6.0)
        if np.sum(mask) > 5:
            if not self.perfil_ok:
                print(">>> [REAL] SENSOR 1: OBJETO FÍSICO DETECTADO (PERFIL OK) <<<")
                self.perfil_ok = True
        
        self.evaluar_semaforo()
        self.new_data = True

    def evaluar_semaforo(self):
        if not self.perfil_ok or self.posicion_x_camion is None:
            self.box_target.color = [1, 0, 0] 
            return

        centro_meta_x = (self.zona_target['min'][0] + self.zona_target['max'][0]) / 2.0
        error = abs(self.posicion_x_camion - centro_meta_x)
        
        if time.time() - self.ultimo_debug > 1.0:
            print(f"   [REAL] DISTANCIA: {self.posicion_x_camion:.2f}m | META: {centro_meta_x:.2f}m | ERROR: {error:.2f}m")
            self.ultimo_debug = time.time()

        TOLERANCIA = 0.5 
        if error <= TOLERANCIA:
            self.box_target.color = [0, 1, 0] 
        elif error <= (TOLERANCIA + 2.0):
            self.box_target.color = [1, 1, 0] 
        else:
            self.box_target.color = [1, 0, 0] 

    # ==========================================
    # UTILIDADES
    # ==========================================
    def cambiar_modo(self, modo):
        print(f">>> CAMBIANDO A MODO {modo} PIES")
        self.modo_actual = modo
        self.zona_target = self.DB_GEOMETRIA[modo]
        self.box_target.min_bound = np.array(self.zona_target['min'])
        self.box_target.max_bound = np.array(self.zona_target['max'])
        self.reset_logica(None)
        self.actualizar_caja = True

    def reset_logica(self, vis):
        self.perfil_ok = False
        self.posicion_x_camion = None
        self.box_target.color = [1, 0, 0]
        print(">>> SISTEMA RESETEADO <<<")

    def crear_camion_urdf(self):
        chassis = o3d.geometry.TriangleMesh.create_box(width=12.0, height=2.4, depth=1.2)
        chassis.compute_vertex_normals(); chassis.paint_uniform_color([0.0, 0.0, 0.8])
        chassis.translate(np.array([-6.0, -1.2, -0.6]))
        
        cabin = o3d.geometry.TriangleMesh.create_box(width=2.5, height=2.4, depth=3.0)
        cabin.compute_vertex_normals(); cabin.paint_uniform_color([0.8, 0.0, 0.0])
        cabin.translate(np.array([-1.25, -1.2, -1.5]))
        cabin.translate(np.array([5.0, 0, 1.5]))

        full_truck = chassis + cabin
        full_truck.translate(np.array([-15.0, 0, 0.8])) 
        return full_truck

    def crear_suelo(self):
        size=40; step=2; points=[]; lines=[]
        start=-size/2; count=int(size/step)+1
        for i in range(count):
            c=start+i*step
            points.append([start,c,0]); points.append([-start,c,0]); lines.append([len(points)-2, len(points)-1])
            points.append([c,start,0]); points.append([c,-start,0]); lines.append([len(points)-2, len(points)-1])
        ls=o3d.geometry.LineSet(); ls.points=o3d.utility.Vector3dVector(points)
        ls.lines=o3d.utility.Vector2iVector(lines); ls.colors=o3d.utility.Vector3dVector([[0.2,0.2,0.2]]*len(lines))
        return ls

    def laser_to_xyz(self, msg, cfg):
        # 1. Obtener datos crudos (Polar)
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        ranges = np.array(msg.ranges)
        
        # Ajuste de seguridad para arrays
        min_len = min(len(angles), len(ranges))
        angles = angles[:min_len]
        ranges = ranges[:min_len]

        # 2. Filtro de Angulo y Distancia (El recorte del abanico)
        lim_min_rad = np.radians(cfg['ang_min'])
        lim_max_rad = np.radians(cfg['ang_max'])

        valid = (ranges > cfg['dist_min']) & (ranges < cfg['dist_max']) & \
                (angles >= lim_min_rad) & (angles <= lim_max_rad)

        r = ranges[valid]
        a = angles[valid]

        if len(r) == 0: return np.zeros((0, 3))

        # 3. Convertir a Cartesianas (X, Y, Z locales del sensor sin rotar)
        # En ROS estándar: X=Frente, Y=Izquierda, Z=Arriba
        x = r * np.cos(a)
        y = r * np.sin(a)
        z = np.zeros_like(x)

        # Empaquetamos en una matriz de puntos (N x 3)
        points = np.vstack((x, y, z)).T

        # =========================================================
        # 4. ROTACIÓN MATRICIAL (LA SOLUCIÓN AL PROBLEMA)
        # =========================================================
        # En lugar de rotar eje por eje manualmente, creamos una Matriz de Rotación 3x3
        # Esto asegura que la rotación interna sea IDÉNTICA a la visual.
        R = o3d.geometry.get_rotation_matrix_from_xyz((cfg['roll'], cfg['pitch'], cfg['yaw']))
        
        # Aplicamos la rotación a todos los puntos de una vez
        points = points @ R.T

        # 5. Traslación final (Posición en la grúa)
        points += np.array(cfg['pos'])

        return points

def main():
    rclpy.init()
    gui = VisualizadorFusionado()
    t = threading.Thread(target=rclpy.spin, args=(gui,), daemon=True)
    t.start()
    try:
        while gui.running:
            # 1. ACTUALIZAR POSICION Y CHEQUEAR CRUCE VIRTUAL
            try:
                now = rclpy.time.Time()
                if gui.tf_buffer.can_transform('map', 'base_link', now):
                    trans = gui.tf_buffer.lookup_transform('map', 'base_link', now)
                    
                    dx = trans.transform.translation.x - gui.truck_x_actual
                    if abs(dx) > 0.001:
                        gui.camion_mesh.translate([dx, 0, 0])
                        gui.vis.update_geometry(gui.camion_mesh)
                        gui.truck_x_actual = trans.transform.translation.x # Guardamos posicion exacta
                    
                    # LLAMAMOS AL NUEVO CHEQUEO
                    gui.chequear_cruce_virtual()

            except: pass

            if gui.new_data:
                gui.pcd_long.points = o3d.utility.Vector3dVector(gui.puntos_long)
                gui.pcd_long.paint_uniform_color([1, 0.5, 0]) 
                gui.pcd_estruc.points = o3d.utility.Vector3dVector(gui.puntos_estruc)
                gui.pcd_estruc.paint_uniform_color([0, 1, 1])
                gui.vis.update_geometry(gui.pcd_long)
                gui.vis.update_geometry(gui.pcd_estruc)
                gui.vis.update_geometry(gui.box_target) 
                gui.new_data = False

            if gui.actualizar_caja:
                gui.vis.update_geometry(gui.box_target)
                gui.actualizar_caja = False
            
            gui.vis.poll_events(); gui.vis.update_renderer(); time.sleep(0.01)
    except KeyboardInterrupt: pass
    finally: gui.vis.destroy_window(); rclpy.shutdown()

if __name__ == '__main__': main()
