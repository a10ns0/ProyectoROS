import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import open3d as o3d
import numpy as np
import threading
import time

class VisualizadorSTS_Simulador(Node):
    def __init__(self):
        super().__init__('visualizador_sts_manual')
        
        # =========================================================
        # A) EL CAMIÓN VIRTUAL (GENERADO DESDE TU URDF)
        # =========================================================
        print(">>> GENERANDO CAMIÓN DESDE ESPECIFICACIÓN URDF... <<<")
        
        # 1. CONSTRUCCIÓN DEL CHASIS (La caja azul)
        # Dimensiones XML: 12.0 x 2.4 x 1.2
        # Open3D crea cajas desde la esquina, así que hay que centrarlas manualmente.
        chassis = o3d.geometry.TriangleMesh.create_box(width=12.0, height=2.4, depth=1.2)
        chassis.compute_vertex_normals()
        chassis.paint_uniform_color([0.0, 0.0, 0.8]) # Material "blue" del URDF
        
        # Mover el pivote al centro del chasis para manejarlo fácil
        chassis.translate(np.array([-6.0, -1.2, -0.6])) # Centrar en (0,0,0) local
        
        # 2. CONSTRUCCIÓN DE LA CABINA (La caja roja)
        # Dimensiones XML: 2.5 x 2.4 x 3.0
        cabin = o3d.geometry.TriangleMesh.create_box(width=2.5, height=2.4, depth=3.0)
        cabin.compute_vertex_normals()
        cabin.paint_uniform_color([0.8, 0.0, 0.0]) # Material "red" del URDF
        
        # Centrar cabina localmente
        cabin.translate(np.array([-1.25, -1.2, -1.5])) 
        
        # 3. ENSAMBLAJE (SOLDADURA VIRTUAL)
        # Según tu URDF, la cabina está desplazada respecto al chasis.
        # Joint 'chassis_to_cabin': xyz="5.0 0 1.5"
        cabin.translate(np.array([5.0, 0, 1.5])) 

        # Fusionamos ambas partes en una sola malla para mover el camión entero
        self.camion_mesh = chassis + cabin
        
        # 4. POSICIONAMIENTO INICIAL EN LA SIMULACIÓN
        # Elevamos todo para que las ruedas (imaginarias) toquen el suelo
        # El chasis tiene altura 1.2, su centro es 0.6. Lo subimos un poco más.
        self.camion_mesh.translate(np.array([0, 0, 0.8]), relative=False)
        
        # ==============================================================================
        # SECCIÓN 1: GEOMETRÍA (CAJAS VIRTUALES)
        # ==============================================================================
        # MODIFICACIÓN SOLICITADA: CAJAS MÁS ALTAS PARA MODO DESCARGA
        # Hemos subido la Z mínima a 1.2m y la máxima a 4.5m.
        # Esto obliga a mirar "arriba" (al contenedor) y no al suelo.
        
        self.DB_GEOMETRIA = {
            '40': { 
                'min': [-6.0, -1.2, 1.0], # <--- Z subió (Antes 0.5)
                'max': [ 6.0,  1.2, 4.5], # <--- Z subió (Antes 4.0)
                'desc': "40 PIES (ALTO)"
            },
            '20': { 
                'min': [-3.0, -1.2, 1.0], # <--- Z subió
                'max': [ 3.0,  1.2, 4.5], # <--- Z subió
                'desc': "20 PIES (ALTO)"
            }
        }
        
        self.ZONA_PERFIL = { 
            'min': [-11, -2.5, 0.0], 
            'max': [ -9.0,  2.5, 5.0] 
        }

        # Estado inicial
        self.modo_actual = '40'
        self.zona_target = self.DB_GEOMETRIA['40']
        
        # =========================================================
        # FILTRO PARA MODO CARGA (CHASIS)
        # =========================================================
        # MODIFICACIÓN SOLICITADA: Límite máximo estricto de 1.7m
        self.CHASIS_VACIO = {
            'alto_min': 0.1,  # 0.6 original # Ignorar suelo/ruedas
            'alto_max': 2.0,  # 1.7 original # <--- CORTE PARA IGNORAR CABINA
            'ancho_min': 0.2, # 2.2 original
        }
        
        self.modo_operacion = "DESCARGA" 

        # ==============================================================================
        # SECCIÓN 2: HARDWARE (SENSORES)
        # ==============================================================================
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

        # ==============================================================================
        # SECCIÓN 3: PARAMETROS PLC
        # ==============================================================================
        self.TIEMPO_CONFIRMACION = 3.0   
        self.PUNTOS_MINIMOS = 5         #30 original
        self.ANCHO_MIN_CAMION = 0.2      #2.0 original 
        self.ALTO_MIN_CAMION = 0.2       #1.5 original

        # ==========================================
        # CONEXIÓN ROS
        # ==========================================
        self.create_subscription(LaserScan, '/scan_distancia', self.callback_longitudinal, 10)
        self.create_subscription(LaserScan, '/scan_estructura', self.callback_estructura, 10)

        # Variables internas (Memoria del sistema)
        self.es_camion_valido = False  
        self.memoria_perfil_ok = False # <--- EL ENCLAVAMIENTO (LATCH)
        self.esta_en_posicion = False  
        self.estado_sistema = "ESPERANDO"
        self.timer_inicio = None
        
        # Buffers gráficos
        self.puntos_long = np.zeros((0, 3))
        self.puntos_estruc = np.zeros((0, 3))
        self.new_data = False
        self.actualizar_caja = False

        # ==========================================
        # INTERFAZ GRÁFICA & BOTONERA
        # ==========================================
        print(">>> VISUALIZADOR MODO MANUAL - ACTUALIZADO <<<")
        print("CONTROLES: [1]=20 Pies | [2]=40 Pies | [C]=Carga (Chasis) | [D]=Descarga (Contenedor)")
        
        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.vis.create_window(window_name="SIMULACION MANUAL STS", width=1280, height=720)
        self.vis.get_render_option().background_color = np.asarray([0.1, 0.1, 0.1])
        self.vis.get_render_option().point_size = 3.0

        # Registrar teclas
        self.vis.register_key_callback(ord("1"), self.btn_set_20)
        self.vis.register_key_callback(ord("2"), self.btn_set_40)
        self.vis.register_key_callback(ord("C"), self.btn_set_carga)
        self.vis.register_key_callback(ord("D"), self.btn_set_descarga)

        # Geometrías
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
        
        # RECUPERANDO LAS ESFERAS (VISUALIZACIÓN DE SENSORES)
        s1 = o3d.geometry.TriangleMesh.create_sphere(radius=0.3)
        s1.paint_uniform_color([1, 0.5, 0]); s1.translate(self.CFG_LONG['pos'])
        self.vis.add_geometry(s1)

        s2 = o3d.geometry.TriangleMesh.create_sphere(radius=0.3)
        s2.paint_uniform_color([0, 1, 1]); s2.translate(self.CFG_ESTRUC['pos'])
        self.vis.add_geometry(s2)
        
        self.configurar_camara()
        self.running = True

    # ==========================================
    # FUNCIONES DE LOS BOTONES
    # ==========================================
    def btn_set_20(self, vis):
        print(">>> BOTÓN: 20 PIES")
        self.modo_actual = '20'
        self.zona_target = self.DB_GEOMETRIA['20']
        self.reset_sistema()
        return False

    def btn_set_40(self, vis):
        print(">>> BOTÓN: 40 PIES")
        self.modo_actual = '40'
        self.zona_target = self.DB_GEOMETRIA['40']
        self.reset_sistema()
        return False

    def btn_set_carga(self, vis):
        print(">>> BOTÓN: CARGA (Buscando Chasis < 1.7m)")
        self.modo_operacion = "CARGA"
        self.reset_sistema()
        return False

    def btn_set_descarga(self, vis):
        print(">>> BOTÓN: DESCARGA (Caja Alta)")
        self.modo_operacion = "DESCARGA"
        self.reset_sistema()
        return False

    def reset_sistema(self):
        self.actualizar_caja = True
        self.estado_sistema = "ESPERANDO"
        self.timer_inicio = None
        self.box_target.color = [1, 0, 0]
        # NOTA: Al cambiar de modo manualmente, también reseteamos la memoria por seguridad
        self.memoria_perfil_ok = False 

    # ==========================================
    # LÓGICA DE SENSORES
    # ==========================================
    def callback_longitudinal(self, msg):
        points = self.laser_to_xyz(msg, self.CFG_LONG)
        self.puntos_long = points
        
        min_b = np.array(self.zona_target['min'])
        max_b = np.array(self.zona_target['max'])
        puntos_validos = 0

        # --- LÓGICA DE POSICIONAMIENTO ---
        if self.modo_operacion == "DESCARGA":
            # MODO CAJA ALTA: La caja ahora está elevada (Z=1.2 a 4.5)
            # El láser debe ver el contenedor en esa zona para validar.
            mask = np.all((points >= min_b) & (points <= max_b), axis=1)
            puntos_validos = np.sum(mask)

        elif self.modo_operacion == "CARGA":
            # MODO CHASIS (FILTRO DE CABINA)
            # 1. Filtramos X e Y (Dentro de la zona de estacionamiento)
            mask_xy = (points[:,0] >= min_b[0]) & (points[:,0] <= max_b[0]) & \
                      (points[:,1] >= min_b[1]) & (points[:,1] <= max_b[1])
            puntos_zona = points[mask_xy]
            
            # 2. FILTRO DE ALTURA ESTRICTO:
            # Solo aceptamos puntos entre 0.6m y 1.7m.
            # Todo lo que sea > 1.7m (Cabina) se descarta.
            mask_z = (puntos_zona[:,2] > self.CHASIS_VACIO['alto_min']) & \
                     (puntos_zona[:,2] < self.CHASIS_VACIO['alto_max'])
            
            puntos_validos = np.sum(mask_z)

        # Umbral de sensibilidad
        umbral = self.PUNTOS_MINIMOS if self.modo_operacion == "DESCARGA" else (self.PUNTOS_MINIMOS / 2)
        
        if puntos_validos > umbral: 
            self.esta_en_posicion = True
        else: 
            self.esta_en_posicion = False
        
        # LÓGICA DE RESET AUTOMÁTICO (Fin de Ciclo)
        if puntos_validos < 5: 
            if self.memoria_perfil_ok == True:
                print(">>> [RESET] ZONA VACÍA - SISTEMA REARMADO <<<")
                self.memoria_perfil_ok = False

        self.evaluar_logica_general()
        self.new_data = True

    def callback_estructura(self, msg):
        points = self.laser_to_xyz(msg, self.CFG_ESTRUC)
        self.puntos_estruc = points

        min_b = np.array(self.ZONA_PERFIL['min'])
        max_b = np.array(self.ZONA_PERFIL['max'])
        mask = np.all((points >= min_b) & (points <= max_b), axis=1)
        objeto_puntos = points[mask]

        if len(objeto_puntos) > 20:
            ancho = np.max(objeto_puntos[:, 1]) - np.min(objeto_puntos[:, 1])
            alto  = np.max(objeto_puntos[:, 2]) - np.min(objeto_puntos[:, 2])
            
            if self.modo_operacion == "DESCARGA":
                # En descarga, esperamos ver algo grande (Contenedor o camión alto)
                if ancho > self.ANCHO_MIN_CAMION and alto > self.ALTO_MIN_CAMION: 
                    self.es_camion_valido = True
                else: 
                    self.es_camion_valido = False
            
            elif self.modo_operacion == "CARGA":
                # En carga, esperamos chasis bajo (< 1.7m)
                condicion_ancho = ancho > self.CHASIS_VACIO['ancho_min']
                condicion_alto  = (alto > 0.6) and (alto < 2.0) # Tolerancia leve en perfil
                
                if condicion_ancho and condicion_alto:
                    self.es_camion_valido = True
                else:
                    self.es_camion_valido = False
        else:
            self.es_camion_valido = False

        # LÓGICA DE ENCLAVAMIENTO (LATCH)
        if self.es_camion_valido == True:
            self.memoria_perfil_ok = True

        self.evaluar_logica_general()
        self.new_data = True

    def evaluar_logica_general(self):
        # COMPUERTA AND CON MEMORIA
        condicion_segura = self.memoria_perfil_ok and self.esta_en_posicion

        if condicion_segura:
            if self.estado_sistema == "ESPERANDO":
                self.estado_sistema = "VALIDANDO"
                self.timer_inicio = time.time()
                self.box_target.color = [1, 1, 0] 

            elif self.estado_sistema == "VALIDANDO":
                if (time.time() - self.timer_inicio) >= self.TIEMPO_CONFIRMACION:
                    self.estado_sistema = "CONFIRMADO"
                    print(f">>> [OK] CAMIÓN {self.modo_actual} PIES LISTO ({self.modo_operacion}) <<<")
                    self.box_target.color = [0, 1, 0] 

        else:
            if self.estado_sistema == "CONFIRMADO": 
                print(">>> CAMIÓN SALIENDO... <<<")
            self.estado_sistema = "ESPERANDO"
            self.timer_inicio = None
            self.box_target.color = [1, 0, 0] 

    # ==========================================
    # UTILIDADES MATEMÁTICAS
    # ==========================================
    def laser_to_xyz(self, msg, cfg):
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        ranges = np.array(msg.ranges)
        min_len = min(len(angles), len(ranges))
        angles = angles[:min_len]; ranges = ranges[:min_len]

        lim_min_rad = np.radians(cfg['ang_min'])
        lim_max_rad = np.radians(cfg['ang_max'])

        valid = (ranges > 0.1) & \
                (ranges > cfg['dist_min']) & (ranges < cfg['dist_max']) & \
                (angles >= lim_min_rad) & (angles <= lim_max_rad)

        r = ranges[valid]; a = angles[valid]
        if len(r) == 0: return np.zeros((0, 3))

        x = r * np.cos(a); y = r * np.sin(a); z = np.zeros_like(x)

        if cfg['roll'] != 0:
            c, s = np.cos(cfg['roll']), np.sin(cfg['roll'])
            y_n = y * c - z * s; z_n = y * s + z * c; y, z = y_n, z_n
        if cfg['pitch'] != 0:
            c, s = np.cos(cfg['pitch']), np.sin(cfg['pitch'])
            x_n = x * c - z * s; z_n = x * s + z * c; x, z = x_n, z_n
        if cfg['yaw'] != 0:
            c, s = np.cos(cfg['yaw']), np.sin(cfg['yaw'])
            x_n = x * c - y * s; y_n = x * s + y * c; x, y = x_n, y_n

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
