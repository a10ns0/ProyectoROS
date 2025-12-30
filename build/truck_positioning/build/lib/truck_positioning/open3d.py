import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import open3d as o3d
import numpy as np
import threading
import time

class VisualizadorSTS_Simulador(Node):
    def __init__(self):
        super().__init__('visualizador_sts_manual')
        
        # ==========================================
        # NUEVO: LISTENER DE TF (Para saber donde esta el camion)
        # ==========================================
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.truck_last_x = -15.0 # Posición inicial conocida

        # ==============================================================================
        # SECCIÓN 1: GEOMETRÍA (CAJAS VIRTUALES)
        # ==============================================================================
        self.DB_GEOMETRIA = {
            '40': { 'min': [-6.0, -1.2, 1.0], 'max': [ 6.0,  1.2, 4.5], 'desc': "40 PIES (ALTO)" },
            '20': { 'min': [-3.0, -1.2, 1.0], 'max': [ 3.0,  1.2, 4.5], 'desc': "20 PIES (ALTO)" }
        }
        self.ZONA_PERFIL = { 'min': [-6.3, -2.5, 0.0], 'max': [ -1.0,  2.5, 5.0] }

        # Estado inicial
        self.modo_actual = '40'
        self.zona_target = self.DB_GEOMETRIA['40']
        
        # Filtros
        self.CHASIS_VACIO = { 'alto_min': 0.1, 'alto_max': 2.0, 'ancho_min': 0.2 }
        self.modo_operacion = "DESCARGA" 

       # ==============================================================================
        # SECCIÓN 2: HARDWARE (SENSORES) - CALIBRADOS CON SIMULADOR
        # ==============================================================================
        
        # --- SENSOR 1: ESTRUCTURA (El que mira el perfil) ---
        # En tu simulador (TruckSimulator) 'sensor1_pos_x' es 0.0 y altura 12.5
        self.CFG_ESTRUC = {
            'pos': [0.0, 0.0, 12.5],  # <--- CORREGIDO: X=0.0 (Origen), Z=12.5 (Altura simulador)
            'dist_min': 0.1, 
            'dist_max': 20.0,         # Rango amplio para ver el suelo desde 12.5m
            'ang_min': -45.0, 
            'ang_max': 45.0, 
            'pitch': np.radians(-90), 
            'yaw': np.radians(0), 
            'roll': np.radians(0)
        }
        
        # --- SENSOR 2: LONGITUDINAL (El que mide distancia/posición) ---
        # En tu simulador (TruckSimulator) 'sensor2_pos_x' es 22.38 y altura 12.5
        self.CFG_LONG = {
            'pos': [22.38, 0.0, 12.5], # <--- CORREGIDO: X=22.38, Z=12.5
            'dist_min': 0.1, 
            'dist_max': 20.0,          # Aumentado a 20m para que alcance a ver el camión desde arriba
            'ang_min': -95.0, 
            'ang_max': 0, 
            'pitch': np.radians(-90), 
            'yaw': np.radians(0), 
            'roll': np.radians(90) 
        }

        # Parametros PLC
        self.TIEMPO_CONFIRMACION = 3.0   
        self.PUNTOS_MINIMOS = 5         
        self.ANCHO_MIN_CAMION = 0.2      
        self.ALTO_MIN_CAMION = 0.2       

        # ==========================================
        # CONEXIÓN ROS
        # ==========================================
        self.create_subscription(LaserScan, '/scan_distancia', self.callback_longitudinal, 10)
        self.create_subscription(LaserScan, '/scan_estructura', self.callback_estructura, 10)

        # Variables internas
        self.es_camion_valido = False  
        self.memoria_perfil_ok = False 
        self.esta_en_posicion = False  
        self.estado_sistema = "ESPERANDO"
        self.timer_inicio = None
        
        # Buffers gráficos
        self.puntos_long = np.zeros((0, 3))
        self.puntos_estruc = np.zeros((0, 3))
        self.new_data = False
        self.actualizar_caja = False

        # ==========================================
        # INTERFAZ GRÁFICA
        # ==========================================
        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.vis.create_window(window_name="SIMULACION MANUAL STS", width=1280, height=720)
        self.vis.get_render_option().background_color = np.asarray([0.1, 0.1, 0.1])
        self.vis.get_render_option().point_size = 3.0

        # Registrar teclas
        self.vis.register_key_callback(ord("1"), self.btn_set_20)
        self.vis.register_key_callback(ord("2"), self.btn_set_40)
        self.vis.register_key_callback(ord("C"), self.btn_set_carga)
        self.vis.register_key_callback(ord("D"), self.btn_set_descarga)

        # Geometrías Básicas
        self.pcd_long = o3d.geometry.PointCloud()
        self.pcd_estruc = o3d.geometry.PointCloud()

        self.box_target = o3d.geometry.AxisAlignedBoundingBox(
            min_bound=np.array(self.zona_target['min']),
            max_bound=np.array(self.zona_target['max'])
        )
        self.box_target.color = [1, 0, 0] 

        self.box_profile = o3d.geometry.AxisAlignedBoundingBox(
            min_bound=np.array(self.ZONA_PERFIL['min']),
            max_bound=np.array(self.ZONA_PERFIL['max'])
        )
        self.box_profile.color = [0, 0, 1] 

        self.vis.add_geometry(self.box_target)
        self.vis.add_geometry(self.box_profile)
        self.vis.add_geometry(self.crear_suelo()) 
        self.vis.add_geometry(self.pcd_long)
        self.vis.add_geometry(self.pcd_estruc)
        
        # Esferas sensores
        s1 = o3d.geometry.TriangleMesh.create_sphere(radius=0.3)
        s1.paint_uniform_color([1, 0.5, 0]); s1.translate(self.CFG_LONG['pos'])
        self.vis.add_geometry(s1)

        s2 = o3d.geometry.TriangleMesh.create_sphere(radius=0.3)
        s2.paint_uniform_color([0, 1, 1]); s2.translate(self.CFG_ESTRUC['pos'])
        self.vis.add_geometry(s2)

        # ==========================================
        # NUEVO: MODELO 3D DEL CAMIÓN (SIMULADO)
        # ==========================================
        # Replicamos las dimensiones de tu URDF
        # 1. Chasis (Azul) - 12m x 2.4m x 1.2m
        self.mesh_chasis = o3d.geometry.TriangleMesh.create_box(width=12.0, height=2.4, depth=1.2)
        self.mesh_chasis.paint_uniform_color([0.0, 0.0, 0.8]) # Azul
        # Ajustar origen del cubo (Open3D crea el cubo desde la esquina, lo centramos en Y)
        self.mesh_chasis.translate([0.0, -1.2, 0.0]) 
        
        # 2. Cabina (Roja) - 2.5m x 2.4m x 3.0m
        self.mesh_cabina = o3d.geometry.TriangleMesh.create_box(width=2.5, height=2.4, depth=3.0)
        self.mesh_cabina.paint_uniform_color([0.8, 0.0, 0.0]) # Rojo
        # La cabina va al final del chasis
        self.mesh_cabina.translate([9.5, -1.2, 0.0])

        # Posicion inicial lejos
        self.mesh_chasis.translate([-15.0, 0, 0])
        self.mesh_cabina.translate([-15.0, 0, 0])

        self.vis.add_geometry(self.mesh_chasis)
        self.vis.add_geometry(self.mesh_cabina)
        
        self.configurar_camara()
        self.running = True

    # ==========================================
    # FUNCIONES DE LOS BOTONES
    # ==========================================
    def btn_set_20(self, vis):
        self.modo_actual = '20'; self.zona_target = self.DB_GEOMETRIA['20']; self.reset_sistema(); return False
    def btn_set_40(self, vis):
        self.modo_actual = '40'; self.zona_target = self.DB_GEOMETRIA['40']; self.reset_sistema(); return False
    def btn_set_carga(self, vis):
        self.modo_operacion = "CARGA"; self.reset_sistema(); return False
    def btn_set_descarga(self, vis):
        self.modo_operacion = "DESCARGA"; self.reset_sistema(); return False

    def reset_sistema(self):
        self.actualizar_caja = True; self.estado_sistema = "ESPERANDO"; self.timer_inicio = None
        self.box_target.color = [1, 0, 0]; self.memoria_perfil_ok = False 

    # ==========================================
    # LÓGICA DE SENSORES
    # ==========================================
    def callback_longitudinal(self, msg):
        points = self.laser_to_xyz(msg, self.CFG_LONG)
        self.puntos_long = points
        
        min_b = np.array(self.zona_target['min']); max_b = np.array(self.zona_target['max'])
        puntos_validos = 0

        if self.modo_operacion == "DESCARGA":
            mask = np.all((points >= min_b) & (points <= max_b), axis=1)
            puntos_validos = np.sum(mask)
        elif self.modo_operacion == "CARGA":
            mask_xy = (points[:,0] >= min_b[0]) & (points[:,0] <= max_b[0]) & (points[:,1] >= min_b[1]) & (points[:,1] <= max_b[1])
            puntos_zona = points[mask_xy]
            mask_z = (puntos_zona[:,2] > self.CHASIS_VACIO['alto_min']) & (puntos_zona[:,2] < self.CHASIS_VACIO['alto_max'])
            puntos_validos = np.sum(mask_z)

        umbral = self.PUNTOS_MINIMOS if self.modo_operacion == "DESCARGA" else (self.PUNTOS_MINIMOS / 2)
        self.esta_en_posicion = (puntos_validos > umbral)
        
        if puntos_validos < 5 and self.memoria_perfil_ok:
            print(">>> [RESET] ZONA VACÍA - SISTEMA REARMADO <<<")
            self.memoria_perfil_ok = False

        self.evaluar_logica_general()
        self.new_data = True

    def callback_estructura(self, msg):
        points = self.laser_to_xyz(msg, self.CFG_ESTRUC)
        self.puntos_estruc = points

        min_b = np.array(self.ZONA_PERFIL['min']); max_b = np.array(self.ZONA_PERFIL['max'])
        mask = np.all((points >= min_b) & (points <= max_b), axis=1)
        objeto_puntos = points[mask]

        if len(objeto_puntos) > 20:
            ancho = np.max(objeto_puntos[:, 1]) - np.min(objeto_puntos[:, 1])
            alto  = np.max(objeto_puntos[:, 2]) - np.min(objeto_puntos[:, 2])
            
            if self.modo_operacion == "DESCARGA":
                self.es_camion_valido = (ancho > self.ANCHO_MIN_CAMION and alto > self.ALTO_MIN_CAMION)
            elif self.modo_operacion == "CARGA":
                self.es_camion_valido = (ancho > self.CHASIS_VACIO['ancho_min'] and alto > 0.6 and alto < 2.0)
        else:
            self.es_camion_valido = False

        if self.es_camion_valido: self.memoria_perfil_ok = True
        self.evaluar_logica_general()
        self.new_data = True

    def evaluar_logica_general(self):
        condicion_segura = self.memoria_perfil_ok and self.esta_en_posicion
        if condicion_segura:
            if self.estado_sistema == "ESPERANDO":
                self.estado_sistema = "VALIDANDO"; self.timer_inicio = time.time(); self.box_target.color = [1, 1, 0] 
            elif self.estado_sistema == "VALIDANDO":
                if (time.time() - self.timer_inicio) >= self.TIEMPO_CONFIRMACION:
                    self.estado_sistema = "CONFIRMADO"; print(f">>> [OK] CAMIÓN {self.modo_actual} LISTO <<<"); self.box_target.color = [0, 1, 0] 
        else:
            if self.estado_sistema == "CONFIRMADO": print(">>> CAMIÓN SALIENDO... <<<")
            self.estado_sistema = "ESPERANDO"; self.timer_inicio = None; self.box_target.color = [1, 0, 0] 

    # ==========================================
    # UTILIDADES
    # ==========================================
    def laser_to_xyz(self, msg, cfg):
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        ranges = np.array(msg.ranges)
        min_len = min(len(angles), len(ranges))
        angles = angles[:min_len]; ranges = ranges[:min_len]

        lim_min_rad = np.radians(cfg['ang_min']); lim_max_rad = np.radians(cfg['ang_max'])
        valid = (ranges > cfg['dist_min']) & (ranges < cfg['dist_max']) & (angles >= lim_min_rad) & (angles <= lim_max_rad)
        r = ranges[valid]; a = angles[valid]
        if len(r) == 0: return np.zeros((0, 3))

        x = r * np.cos(a); y = r * np.sin(a); z = np.zeros_like(x)
        if cfg['roll'] != 0: c, s = np.cos(cfg['roll']), np.sin(cfg['roll']); y_n = y * c - z * s; z_n = y * s + z * c; y, z = y_n, z_n
        if cfg['pitch'] != 0: c, s = np.cos(cfg['pitch']), np.sin(cfg['pitch']); x_n = x * c - z * s; z_n = x * s + z * c; x, z = x_n, z_n
        if cfg['yaw'] != 0: c, s = np.cos(cfg['yaw']), np.sin(cfg['yaw']); x_n = x * c - y * s; y_n = x * s + y * c; x, y = x_n, y_n
        return np.vstack((x + cfg['pos'][0], y + cfg['pos'][1], z + cfg['pos'][2])).T

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

    def configurar_camara(self):
        ctr = self.vis.get_view_control()
        ctr.set_lookat([0,0,0]); ctr.set_front([0.0, -0.8, 0.8]); ctr.set_up([0,0,1]); ctr.set_zoom(0.5)

def main(args=None):
    rclpy.init(args=args)
    gui = VisualizadorSTS_Simulador()
    t = threading.Thread(target=rclpy.spin, args=(gui,), daemon=True)
    t.start()
    try:
        while gui.running:
            # =========================================================
            # NUEVO: ACTUALIZACIÓN DE POSICIÓN DEL CAMIÓN (ANIMACIÓN)
            # =========================================================
            try:
                # Buscamos la transformación desde 'map' a 'base_link' (el camión)
                # El '0' indica que queremos la transformación más reciente posible
                now = rclpy.time.Time()
                trans = gui.tf_buffer.lookup_transform('map', 'base_link', now)
                
                # Obtenemos la posición X actual del camión simulado
                current_x = trans.transform.translation.x
                
                # Calculamos cuánto se movió desde el último frame
                delta_x = current_x - gui.truck_last_x
                
                if abs(delta_x) > 0.001: # Si se movió
                    # Movemos las mallas 3D
                    gui.mesh_chasis.translate([delta_x, 0, 0])
                    gui.mesh_cabina.translate([delta_x, 0, 0])
                    
                    # Actualizamos en el visualizador
                    gui.vis.update_geometry(gui.mesh_chasis)
                    gui.vis.update_geometry(gui.mesh_cabina)
                    
                    # Guardamos la posición para la siguiente resta
                    gui.truck_last_x = current_x
            except TransformException:
                # Al principio puede fallar unos ms mientras llega la info
                pass

            # Actualización de Nube de Puntos
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
                gui.box_target.min_bound = np.array(gui.zona_target['min'])
                gui.box_target.max_bound = np.array(gui.zona_target['max'])
                gui.vis.update_geometry(gui.box_target)
                gui.actualizar_caja = False
            
            gui.vis.poll_events()
            gui.vis.update_renderer()
            time.sleep(0.01)
    except KeyboardInterrupt: pass
    finally: gui.vis.destroy_window(); rclpy.shutdown()

if __name__ == '__main__': main()
