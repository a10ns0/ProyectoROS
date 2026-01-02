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
import json  # <--- NECESARIO PARA LEER LA BASE DE DATOS

class SistemaValidacionConectado(Node):
    def __init__(self):
        super().__init__('sistema_validacion_conectado')
        
        # =========================================================
        # 1. ESCALADO (1.5 METROS)
        # =========================================================
        DISTANCIA_REAL_ORIGINAL = 20.4
        DISTANCIA_MAQUETA = 1.5 
        self.SCALE = DISTANCIA_MAQUETA / DISTANCIA_REAL_ORIGINAL # aprox 0.0735
        
        # =========================================================
        # 2. CALIBRACIÓN PARA TU CAJA
        # =========================================================
        self.MIN_ANCHO_CAMION = 2.0 * self.SCALE 
        self.MIN_ALTO_CAMION  = 1.3 * self.SCALE 
        
        self.H_FILTRO_CONTENEDOR   = 2.45 * self.SCALE  
        self.H_FILTRO_CHASIS_HIGH  = 1.8 * self.SCALE   
        self.H_FILTRO_CHASIS_LOW   = 0.5 * self.SCALE   

        print(f"=== SISTEMA CONECTADO A API (Escala {self.SCALE:.4f}) ===")

        # =========================================================
        # 3. ZONAS Y LÓGICA
        # =========================================================
        self.perfil_ok = False          
        self.posicion_detectada = None
        self.ultimo_debug = 0
        
        # VARIABLES DE ESTADO (Ahora se llenan solas por API)
        self.db_spreader_size = '40'
        self.db_twistlocks = 'CLOSE'
        self.modo_operacion = 'DESCARGA_DEL_BARCO'

        self.DB_GEOMETRIA = {
            '40': { 'min': [-6.0*self.SCALE, -1.2*self.SCALE, 0.0], 'max': [ 6.0*self.SCALE,  1.2*self.SCALE, 5.0*self.SCALE] },
            '20': { 'min': [-3.0*self.SCALE, -1.2*self.SCALE, 0.0], 'max': [ 3.0*self.SCALE,  1.2*self.SCALE, 5.0*self.SCALE] }
        }
        self.zona_target = self.DB_GEOMETRIA['40']
        
        self.ZONA_PERFIL = {
            'min': [-11.0*self.SCALE, -2.5*self.SCALE, 0.0], 
            'max': [ -9.0*self.SCALE,  2.5*self.SCALE, 5.0*self.SCALE]
        }

        # Configuración Sensores
        self.Z_SENSOR = 12.5 * self.SCALE
        self.X_SENSOR = 10.2 * self.SCALE 

        self.CFG_LONG = {
            'pos': [self.X_SENSOR, 0.0, self.Z_SENSOR], 
            'dist_min': 0.05, 'dist_max': 40 * self.SCALE,
            'ang_min': -90.0, 'ang_max': 0, 
            'pitch': np.radians(0), 'yaw': np.radians(-90), 'roll': np.radians(90)
        }
        
        self.CFG_ESTRUC = {
            'pos': [-self.X_SENSOR, 11.19*self.SCALE, self.Z_SENSOR],
            'dist_min': 0.05, 'dist_max': 40 * self.SCALE,
            'ang_min': -45.0, 'ang_max': 45.0,
            'pitch': np.radians(-90), 'yaw': np.radians(180), 'roll': np.radians(0)
        }

        # --- SUSCRIPCIONES ROS ---
        self.create_subscription(LaserScan, '/scan_distancia', self.cb_longitudinal, 10)
        self.create_subscription(LaserScan, '/scan_estructura', self.cb_estructura, 10)
        
        # NUEVO: Suscripción a la Base de Datos / API
        self.create_subscription(String, 'grua/estado_completo', self.cb_database, 10)
        
        self.puntos_long = np.zeros((0, 3))
        self.puntos_estruc = np.zeros((0, 3))
        self.new_data = False
        self.actualizar_geometria = False # Bandera para actualizar caja roja

        self.init_gui()
        self.running = True

    def init_gui(self):
        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.vis.create_window("SISTEMA CONECTADO API", 1280, 720)
        self.vis.get_render_option().point_size = 5.0
        self.vis.get_render_option().background_color = np.asarray([0.3, 0.3, 0.3]) 

        # Teclas de Debug (Por si la API falla, puedes forzar manual)
        self.vis.register_key_callback(ord("R"), self.reset_sistema)
        self.vis.register_key_callback(ord("B"), self.teletransportar_a_azul)

        self.pcd_long = o3d.geometry.PointCloud()
        self.pcd_estruc = o3d.geometry.PointCloud()

        self.box_target = o3d.geometry.AxisAlignedBoundingBox(
            min_bound=np.array(self.zona_target['min']), max_bound=np.array(self.zona_target['max']))
        self.box_target.color = [1, 0, 0]

        self.box_profile = o3d.geometry.AxisAlignedBoundingBox(
            min_bound=np.array(self.ZONA_PERFIL['min']), max_bound=np.array(self.ZONA_PERFIL['max']))
        self.box_profile.color = [0, 0, 1] 

        self.vis.add_geometry(self.crear_referencia_altura(self.H_FILTRO_CHASIS_HIGH, [1, 1, 0])) 
        self.vis.add_geometry(self.crear_referencia_altura(self.H_FILTRO_CONTENEDOR, [0, 1, 1])) 

        self.vis.add_geometry(self.box_target); self.vis.add_geometry(self.box_profile)
        self.vis.add_geometry(self.crear_suelo_escalado()) 
        self.vis.add_geometry(self.pcd_long); self.vis.add_geometry(self.pcd_estruc)
        
        s1 = o3d.geometry.TriangleMesh.create_sphere(radius=0.3 * self.SCALE)
        s1.translate(self.CFG_LONG['pos']); s1.paint_uniform_color([1, 0.5, 0])
        s2 = o3d.geometry.TriangleMesh.create_sphere(radius=0.3 * self.SCALE)
        s2.translate(self.CFG_ESTRUC['pos']); s2.paint_uniform_color([0, 1, 1])
        self.vis.add_geometry(s1); self.vis.add_geometry(s2)

        ctr = self.vis.get_view_control()
        ctr.set_lookat([0,0,0]); ctr.set_front([0.0, -0.5, 0.8]); ctr.set_zoom(0.25)

    # ==========================================
    # NUEVA LÓGICA DE BASE DE DATOS (CALLBACK)
    # ==========================================
    def cb_database(self, msg):
        try:
            # 1. Decodificar JSON
            data = json.loads(msg.data)
            
            # 2. Actualizar Spreader (Tamaño)
            # La API puede mandar '20', '40', o numeros 20, 40. Lo convertimos a string.
            if 'spreaderSize' in data:
                nuevo_tamano = str(data['spreaderSize'])
                # Solo si es '20' o '40' y es diferente al actual, actualizamos
                if nuevo_tamano in ['20', '40'] and nuevo_tamano != self.db_spreader_size:
                    self.db_spreader_size = nuevo_tamano
                    self.actualizar_geometria_spreader()
            
            # 3. Actualizar Twistlocks y Modo de Operación
            if 'spreaderTwistlock' in data:
                val = data['spreaderTwistlock']
                
                # Mapeo de valores posibles de la API a nuestra lógica
                # Ajusta esto según lo que realmente mande tu API (True, 1, "LOCKED", "CLOSE", etc)
                es_cerrado = (val == "CLOSE") or (val == "LOCKED") or (val == True) or (val == 1)
                
                if es_cerrado:
                    self.db_twistlocks = 'CLOSE'
                    self.modo_operacion = 'DESCARGA_DEL_BARCO'
                else:
                    self.db_twistlocks = 'OPEN'
                    self.modo_operacion = 'CARGA_AL_BARCO'

        except json.JSONDecodeError:
            print("Error decodificando JSON de la API")

    def actualizar_geometria_spreader(self):
        # Actualiza las coordenadas de la caja roja según 20 o 40 pies
        self.zona_target = self.DB_GEOMETRIA[self.db_spreader_size]
        self.box_target.min_bound = np.array(self.zona_target['min'])
        self.box_target.max_bound = np.array(self.zona_target['max'])
        self.actualizar_geometria = True # Avisa al loop principal
        print(f">>> API: Spreader cambiado a {self.db_spreader_size} Pies")

    # ==========================================
    # RESTO DE LA LÓGICA (IGUAL QUE ANTES)
    # ==========================================
    def reset_sistema(self, vis):
        self.perfil_ok = False
        self.box_profile.color = [0, 0, 1]
        self.box_target.color = [1, 0, 0]
        print(">>> SISTEMA RESETEADO")

    def cb_estructura(self, msg):
        points = self.laser_to_xyz(msg, self.CFG_ESTRUC)
        self.puntos_estruc = points
        
        if self.perfil_ok:
            self.new_data = True
            return

        min_b = np.array(self.ZONA_PERFIL['min']); max_b = np.array(self.ZONA_PERFIL['max'])
        mask_box = np.all((points >= min_b) & (points <= max_b), axis=1)
        puntos_en_caja = points[mask_box]

        if len(puntos_en_caja) > 10:
            ancho_obj = np.max(puntos_en_caja[:,1]) - np.min(puntos_en_caja[:,1])
            alto_obj  = np.max(puntos_en_caja[:,2]) - np.min(puntos_en_caja[:,2])
            
            es_camion = (ancho_obj > self.MIN_ANCHO_CAMION) and (alto_obj > self.MIN_ALTO_CAMION)
            
            if es_camion:
                self.perfil_ok = True
                self.box_profile.color = [0, 1, 1] 
                print(f">>> [PERFIL OK] Detectado: {ancho_obj*100:.1f}x{alto_obj*100:.1f}cm")

        self.new_data = True

    def cb_longitudinal(self, msg):
        points = self.laser_to_xyz(msg, self.CFG_LONG)
        self.puntos_long = points
        
        if not self.perfil_ok:
            self.posicion_detectada = None
            self.evaluar_semaforo(); self.new_data = True; return

        ancho_calle = 3.5 * self.SCALE
        mask_calle = (points[:,1] > -ancho_calle) & (points[:,1] < ancho_calle)
        mask_altura = None

        if self.modo_operacion == 'CARGA_AL_BARCO':
            mask_altura = (points[:,2] > self.H_FILTRO_CONTENEDOR)
        elif self.modo_operacion == 'DESCARGA_DEL_BARCO':
            mask_altura = (points[:,2] > self.H_FILTRO_CHASIS_LOW) & \
                          (points[:,2] < self.H_FILTRO_CHASIS_HIGH)

        pts_validos = points[mask_calle & mask_altura]

        if len(pts_validos) > 5:
            self.posicion_detectada = np.mean(pts_validos[:,0])
        else:
            self.posicion_detectada = None

        self.evaluar_semaforo()
        self.new_data = True

    def evaluar_semaforo(self):
        if not self.perfil_ok or self.posicion_detectada is None:
            self.box_target.color = [1, 0, 0]; return

        centro_meta_x = (self.zona_target['min'][0] + self.zona_target['max'][0]) / 2.0
        error = abs(self.posicion_detectada - centro_meta_x)
        
        if time.time() - self.ultimo_debug > 1.0:
            print(f"[{self.modo_operacion}] Dist: {self.posicion_detectada:.2f}m | Err: {error:.2f}m")
            self.ultimo_debug = time.time()

        TOLERANCIA = 0.5 * self.SCALE 
        if error <= TOLERANCIA:
            self.box_target.color = [0, 1, 0] 
        elif error <= (TOLERANCIA * 4):
            self.box_target.color = [1, 1, 0] 
        else:
            self.box_target.color = [1, 0, 0] 

    def teletransportar_a_azul(self, vis):
        ctr = self.vis.get_view_control(); ctr.set_lookat(self.CFG_ESTRUC['pos']); ctr.set_zoom(0.05); return False

    def laser_to_xyz(self, msg, cfg):
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        ranges = np.array(msg.ranges)
        min_len = min(len(angles), len(ranges)); angles = angles[:min_len]; ranges = ranges[:min_len]
        lim_min_rad = np.radians(cfg['ang_min']); lim_max_rad = np.radians(cfg['ang_max'])
        valid = (ranges > cfg['dist_min']) & (ranges < cfg['dist_max']) & (angles >= lim_min_rad) & (angles <= lim_max_rad)
        r = ranges[valid]; a = angles[valid]
        if len(r) == 0: return np.zeros((0, 3))
        x = r * np.cos(a); y = r * np.sin(a); z = np.zeros_like(x)
        points = np.vstack((x, y, z)).T
        R = o3d.geometry.get_rotation_matrix_from_xyz((cfg['roll'], cfg['pitch'], cfg['yaw']))
        points = points @ R.T; points += np.array(cfg['pos'])
        return points

    def crear_suelo_escalado(self):
        size=40*self.SCALE; step=2*self.SCALE; points=[]; lines=[]; start=-size/2; count=int(size/step)+1
        for i in range(count):
            c=start+i*step; points.append([start,c,0]); points.append([-start,c,0]); lines.append([len(points)-2, len(points)-1])
            points.append([c,start,0]); points.append([c,-start,0]); lines.append([len(points)-2, len(points)-1])
        ls=o3d.geometry.LineSet(); ls.points=o3d.utility.Vector3dVector(points)
        ls.lines=o3d.utility.Vector2iVector(lines); ls.colors=o3d.utility.Vector3dVector([[0.2,0.2,0.2]]*len(lines))
        return ls

    def crear_referencia_altura(self, z, color):
        points = [[-5*self.SCALE, -2*self.SCALE, z], [ 5*self.SCALE, -2*self.SCALE, z],[-5*self.SCALE, 2*self.SCALE, z], [ 5*self.SCALE, 2*self.SCALE, z]]
        lines = [[0,1], [1,3], [3,2], [2,0]]; ls = o3d.geometry.LineSet(); ls.points = o3d.utility.Vector3dVector(points); ls.lines = o3d.utility.Vector2iVector(lines); ls.paint_uniform_color(color); return ls

def main():
    rclpy.init()
    gui = SistemaValidacionConectado()
    t = threading.Thread(target=rclpy.spin, args=(gui,), daemon=True)
    t.start()
    try:
        while gui.running:
            if gui.new_data:
                gui.pcd_long.points = o3d.utility.Vector3dVector(gui.puntos_long); gui.pcd_long.paint_uniform_color([1, 0.5, 0]) 
                gui.pcd_estruc.points = o3d.utility.Vector3dVector(gui.puntos_estruc); gui.pcd_estruc.paint_uniform_color([0, 1, 1])
                gui.vis.update_geometry(gui.pcd_long); gui.vis.update_geometry(gui.pcd_estruc)
                gui.vis.update_geometry(gui.box_target); gui.vis.update_geometry(gui.box_profile)
                gui.new_data = False
            
            # Actualización de GUI por cambio de Base de Datos
            if gui.actualizar_geometria:
                gui.vis.update_geometry(gui.box_target)
                gui.actualizar_geometria = False

            gui.vis.poll_events(); gui.vis.update_renderer(); time.sleep(0.01)
    except KeyboardInterrupt: pass
    finally: gui.vis.destroy_window(); rclpy.shutdown()

if __name__ == '__main__': main()
