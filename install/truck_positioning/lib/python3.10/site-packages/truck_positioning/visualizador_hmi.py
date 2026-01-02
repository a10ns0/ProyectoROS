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

class SistemaMaquetaPro(Node):
    def __init__(self):
        super().__init__('sistema_maqueta_pro')
        
        # =========================================================
        # 1. CÁLCULO DE ESCALA (1.5 METROS DE SEPARACIÓN)
        # =========================================================
        DISTANCIA_REAL_ORIGINAL = 20.4
        DISTANCIA_MAQUETA = 1.5 
        
        # Factor de Escala (~0.0735)
        self.SCALE = DISTANCIA_MAQUETA / DISTANCIA_REAL_ORIGINAL
        
        # LÍMITE CRÍTICO: 1.7 Metros Reales escalados a la maqueta
        # Si la escala es 0.07, esto será aprox 12.5 cm de alto en tu mesa
        self.ALTURA_MAX_CHASIS = 1.7 * self.SCALE 
        
        print("="*60)
        print(f"=== SISTEMA STS MAQUETA PRO (Separación 1.5m) ===")
        print(f"Escala calculada: {self.SCALE:.4f}")
        print(f"Altura de corte (Chasis vs Contenedor): {self.ALTURA_MAX_CHASIS*100:.1f} cm")
        print("="*60)

        # =========================================================
        # 2. DEFINICIÓN DE ZONAS (20 y 40 Pies)
        # =========================================================
        self.DB_GEOMETRIA = {
            '40': { 
                'min': [-6.0*self.SCALE, -1.2*self.SCALE, 0.0], 
                'max': [ 6.0*self.SCALE,  1.2*self.SCALE, 4.5*self.SCALE], 
                'desc': "40 PIES"
            },
            '20': { 
                'min': [-3.0*self.SCALE, -1.2*self.SCALE, 0.0], 
                'max': [ 3.0*self.SCALE,  1.2*self.SCALE, 4.5*self.SCALE], 
                'desc': "20 PIES"
            }
        }
        
        # Estado Inicial
        self.modo_tamano = '40'
        self.modo_tipo   = 'CHASIS' # 'CHASIS' (Vacío) o 'CONTENEDOR' (Lleno)
        self.zona_target = self.DB_GEOMETRIA['40']
        
        self.posicion_x_camion = None
        self.ultimo_debug = 0

        # =========================================================
        # 3. CONFIGURACIÓN DE SENSORES
        # =========================================================
        # SENSOR 2 (Longitudinal) - DERECHA
        self.CFG_LONG = {
            'pos': [10.2 * self.SCALE, 0.0, 12.5 * self.SCALE], 
            'dist_min': 0.05, 'dist_max': 40 * self.SCALE,
            'ang_min': -90.0, 'ang_max': 90.0, 
            'pitch': np.radians(0), 
            'yaw': np.radians(180), 
            'roll': np.radians(90)
        }

        # SENSOR 1 (Estructura) - IZQUIERDA
        self.CFG_ESTRUC = {
            'pos': [-10.2 * self.SCALE, 11.19 * self.SCALE, 12.5 * self.SCALE],
            'dist_min': 0.05, 'dist_max': 70 * self.SCALE,
            'ang_min': -45.0, 'ang_max': 45.0,
            'pitch': np.radians(-90), 'yaw': np.radians(180), 'roll': np.radians(0)
        }

        # Camión Virtual
        self.truck_x_virtual = -15.0 * self.SCALE 

        # ROS Setup
        self.create_subscription(LaserScan, '/scan_distancia', self.cb_longitudinal, 10)
        self.create_subscription(LaserScan, '/scan_estructura', self.cb_estructura, 10)
        self.pub_cmd = self.create_publisher(String, '/truck_cmd', 10)
        
        self.puntos_long = np.zeros((0, 3))
        self.puntos_estruc = np.zeros((0, 3))
        self.new_data = False
        self.actualizar_caja = False
        
        self.init_gui()
        self.running = True

    def init_gui(self):
        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.vis.create_window("CONTROL MAQUETA STS", 1280, 720)
        self.vis.get_render_option().point_size = 5.0

        # --- CONTROLES TECLADO ---
        self.vis.register_key_callback(ord("1"), lambda v: self.set_tamano('20'))
        self.vis.register_key_callback(ord("2"), lambda v: self.set_tamano('40'))
        self.vis.register_key_callback(ord("V"), lambda v: self.set_tipo('CHASIS'))     # Vacío
        self.vis.register_key_callback(ord("C"), lambda v: self.set_tipo('CONTENEDOR')) # Con Carga
        
        # Mover camión virtual (Referencia)
        self.vis.register_key_callback(ord("W"), lambda v: self.mover_camion(0.05))
        self.vis.register_key_callback(ord("S"), lambda v: self.mover_camion(-0.05))

        # --- VISUALIZACIÓN ---
        self.pcd_long = o3d.geometry.PointCloud()
        self.pcd_estruc = o3d.geometry.PointCloud()

        self.box_target = o3d.geometry.AxisAlignedBoundingBox(
            min_bound=np.array(self.zona_target['min']), max_bound=np.array(self.zona_target['max']))
        self.box_target.color = [1, 0, 0]

        self.vis.add_geometry(self.box_target)
        self.vis.add_geometry(self.crear_suelo_escalado()) 
        self.vis.add_geometry(self.pcd_long)
        self.vis.add_geometry(self.pcd_estruc)
        
        # Esferas Sensores
        s1 = o3d.geometry.TriangleMesh.create_sphere(radius=0.3 * self.SCALE)
        s1.translate(self.CFG_LONG['pos']); s1.paint_uniform_color([1, 0.5, 0])
        s2 = o3d.geometry.TriangleMesh.create_sphere(radius=0.3 * self.SCALE)
        s2.translate(self.CFG_ESTRUC['pos']); s2.paint_uniform_color([0, 1, 1])
        self.vis.add_geometry(s1); self.vis.add_geometry(s2)

        # Camión 3D
        self.camion_mesh = self.crear_camion_urdf()
        self.vis.add_geometry(self.camion_mesh)

        # Texto de estado en consola
        print("\nCONTROLES ACTIVOS:")
        print("[1] 20 Pies  | [2] 40 Pies")
        print("[V] Chasis Vacío (< 1.7m) | [C] Contenedor (> 1.7m)")
        print("[W/S] Mover Camión Virtual\n")

        ctr = self.vis.get_view_control()
        ctr.set_lookat([0,0,0]); ctr.set_front([0.0, -0.5, 0.8]); ctr.set_zoom(0.25)

    # ==========================================
    # LÓGICA DE FILTRADO (LA CLAVE)
    # ==========================================
    def cb_longitudinal(self, msg):
        points = self.laser_to_xyz(msg, self.CFG_LONG)
        self.puntos_long = points
        
        # 1. Filtro Espacial (Solo lo que está en la "calle")
        ancho_calle = 3.5 * self.SCALE
        mask_calle = (points[:,1] > -ancho_calle) & (points[:,1] < ancho_calle)
        
        # 2. Filtro de Altura según MODO (Chasis vs Contenedor)
        # Z > 0.02 evita el suelo de la mesa
        if self.modo_tipo == 'CHASIS':
            # Buscamos objetos BAJOS (Menor a 1.7m real escalado)
            mask_altura = (points[:,2] > 0.02) & (points[:,2] < self.ALTURA_MAX_CHASIS)
        else:
            # Buscamos objetos ALTOS (Mayor a 1.7m real escalado)
            mask_altura = (points[:,2] >= self.ALTURA_MAX_CHASIS)

        # Aplicamos ambos filtros
        mask_total = mask_calle & mask_altura
        pts_validos = points[mask_total]

        if len(pts_validos) > 5:
            self.posicion_x_camion = np.mean(pts_validos[:,0])
        else:
            self.posicion_x_camion = None

        self.evaluar_semaforo()
        self.new_data = True

    def cb_estructura(self, msg):
        points = self.laser_to_xyz(msg, self.CFG_ESTRUC)
        self.puntos_estruc = points
        self.new_data = True

    def evaluar_semaforo(self):
        if self.posicion_x_camion is None:
            self.box_target.color = [1, 0, 0] 
            return

        centro_meta_x = (self.zona_target['min'][0] + self.zona_target['max'][0]) / 2.0
        error = abs(self.posicion_x_camion - centro_meta_x)
        
        # Debug en terminal
        if time.time() - self.ultimo_debug > 1.0:
            modo_str = "VACIO (Buscando < 1.7m)" if self.modo_tipo == 'CHASIS' else "LLENO (Buscando > 1.7m)"
            print(f"[{self.modo_tamano}' | {modo_str}] Pos: {self.posicion_x_camion:.3f}m | Error: {error:.3f}m")
            self.ultimo_debug = time.time()

        TOLERANCIA = 0.5 * self.SCALE 
        if error <= TOLERANCIA:
            self.box_target.color = [0, 1, 0] # VERDE
        elif error <= (TOLERANCIA * 4):
            self.box_target.color = [1, 1, 0] # AMARILLO
        else:
            self.box_target.color = [1, 0, 0] # ROJO

    # ==========================================
    # HELPERS
    # ==========================================
    def set_tamano(self, t):
        self.modo_tamano = t
        self.zona_target = self.DB_GEOMETRIA[t]
        self.box_target.min_bound = np.array(self.zona_target['min'])
        self.box_target.max_bound = np.array(self.zona_target['max'])
        self.actualizar_caja = True
        print(f">>> MODO: {t} PIES")

    def set_tipo(self, t):
        self.modo_tipo = t
        print(f">>> MODO: CAMIÓN {t}")

    def mover_camion(self, dx):
        self.truck_x_virtual += dx
        self.camion_mesh.translate([dx, 0, 0])
        self.vis.update_geometry(self.camion_mesh)

    def crear_camion_urdf(self):
        chassis = o3d.geometry.TriangleMesh.create_box(width=12.0*self.SCALE, height=2.4*self.SCALE, depth=1.2*self.SCALE)
        chassis.compute_vertex_normals(); chassis.paint_uniform_color([0.0, 0.0, 0.8])
        chassis.translate(np.array([-6.0*self.SCALE, -1.2*self.SCALE, 0]))
        
        cabin = o3d.geometry.TriangleMesh.create_box(width=2.5*self.SCALE, height=2.4*self.SCALE, depth=3.0*self.SCALE)
        cabin.compute_vertex_normals(); cabin.paint_uniform_color([0.8, 0.0, 0.0])
        cabin.translate(np.array([-1.25*self.SCALE, -1.2*self.SCALE, -1.5*self.SCALE]))
        cabin.translate(np.array([5.0*self.SCALE, 0, 1.5*self.SCALE]))

        full_truck = chassis + cabin
        full_truck.translate(np.array([self.truck_x_virtual, 0, 0.8*self.SCALE])) 
        return full_truck

    def crear_suelo_escalado(self):
        size=40 * self.SCALE; step=2 * self.SCALE; points=[]; lines=[]
        start=-size/2; count=int(size/step)+1
        for i in range(count):
            c=start+i*step
            points.append([start,c,0]); points.append([-start,c,0]); lines.append([len(points)-2, len(points)-1])
            points.append([c,start,0]); points.append([c,-start,0]); lines.append([len(points)-2, len(points)-1])
        ls=o3d.geometry.LineSet(); ls.points=o3d.utility.Vector3dVector(points)
        ls.lines=o3d.utility.Vector2iVector(lines); ls.colors=o3d.utility.Vector3dVector([[0.2,0.2,0.2]]*len(lines))
        return ls

    def laser_to_xyz(self, msg, cfg):
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        ranges = np.array(msg.ranges)
        min_len = min(len(angles), len(ranges))
        angles = angles[:min_len]; ranges = ranges[:min_len]

        lim_min_rad = np.radians(cfg['ang_min'])
        lim_max_rad = np.radians(cfg['ang_max'])

        valid = (ranges > cfg['dist_min']) & (ranges < cfg['dist_max']) & \
                (angles >= lim_min_rad) & (angles <= lim_max_rad)

        r = ranges[valid]; a = angles[valid]
        if len(r) == 0: return np.zeros((0, 3))

        x = r * np.cos(a); y = r * np.sin(a); z = np.zeros_like(x)
        points = np.vstack((x, y, z)).T
        
        R = o3d.geometry.get_rotation_matrix_from_xyz((cfg['roll'], cfg['pitch'], cfg['yaw']))
        points = points @ R.T
        points += np.array(cfg['pos'])
        return points

def main():
    rclpy.init()
    gui = SistemaMaquetaPro()
    t = threading.Thread(target=rclpy.spin, args=(gui,), daemon=True)
    t.start()
    try:
        while gui.running:
            if gui.new_data:
                gui.pcd_long.points = o3d.utility.Vector3dVector(gui.puntos_long)
                gui.pcd_long.paint_uniform_color([1, 0.5, 0]) 
                gui.pcd_estruc.points = o3d.utility.Vector3dVector(gui.puntos_estruc)
                gui.pcd_estruc.paint_uniform_color([0, 1, 1])
                
                gui.vis.update_geometry(gui.pcd_long)
                gui.vis.update_geometry(gui.pcd_estruc)
                gui.vis.update_geometry(gui.box_target)
                
                if gui.actualizar_caja:
                    gui.vis.update_geometry(gui.box_target)
                    gui.actualizar_caja = False
                    
                gui.new_data = False
            
            gui.vis.poll_events(); gui.vis.update_renderer(); time.sleep(0.01)
    except KeyboardInterrupt: pass
    finally: gui.vis.destroy_window(); rclpy.shutdown()

if __name__ == '__main__': main()
