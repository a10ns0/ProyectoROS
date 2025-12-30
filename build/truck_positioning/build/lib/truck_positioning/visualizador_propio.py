import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import open3d as o3d
import numpy as np
import threading
import time
import json

class VisualizadorSTS(Node):
    def __init__(self):
        super().__init__('visualizador_sts_realtime')
        
        # ==============================================================================
        # SECCIÓN 1: GEOMETRÍA (CAJAS VIRTUALES)
        # ==============================================================================
        # Aquí defines las dimensiones de las zonas donde el camión debe estacionarse.
        # Formato: [X (Largo), Y (Ancho), Z (Alto)] en METROS.
        
        self.DB_GEOMETRIA = {
            '40': { # Configuración para Spreader de 40 pies
                'min': [-6.0, -1.2, 0.5], # Esquina inferior izquierda trasera
                'max': [ 6.0,  1.2, 4.0], # Esquina superior derecha delantera
                'desc': "40 PIES"
            },
            '20': { # Configuración para Spreader de 20 pies
                'min': [-3.0, -1.2, 0.5], 
                'max': [ 3.0,  1.2, 4.0], 
                'desc': "20 PIES"
            }
        }
        
        # ZONA DE PERFIL (Puerta virtual):
        # Usada por el segundo sensor para ver si el objeto es ancho (camión) o delgado (persona).
        self.ZONA_PERFIL = { 
            'min': [-6.3, -2.5, 0.0], 
            'max': [ -1.0,  2.5, 5.0] 
        }

        # Estado inicial (Por defecto 40 pies)
        self.modo_actual = '40'
        self.zona_target = self.DB_GEOMETRIA['40']

# =========================================================
        # NUEVO: VARIABLES PARA MODO CARGA (CHASIS VACÍO)
        # =========================================================
        # Configuración para validar Chasis Vacíos (MODO CARGA)
        self.CHASIS_VACIO = {
            'alto_min': 0.8,  # Altura de la viga del chasis
            'alto_max': 1.6,  # Menos que la cabina
            'ancho_min': 2.2, 
        }
        
        # Variable para saber qué estamos buscando: "CARGA" o "DESCARGA"
        self.modo_operacion = "DESCARGA" 
        # =========================================================
        # ==============================================================================
        # SECCIÓN 2: CONFIGURACIÓN DE HARDWARE (SENSORES) 🔥 <--- MODIFICAR AQUÍ
        # ==============================================================================
        # Ajusta aquí la posición física y rotación de tus sensores SICK.
        # NOTA: Los ángulos están en GRADOS para que sea fácil entender.
        
        # --- SENSOR 1: LONGITUDINAL (IP .100 aprox) ---
        # Este sensor mira a lo largo para ver si el camión llegó al punto de frenado.
        self.CFG_LONG = {
            # 1. POSICIÓN FÍSICA (Metros respecto al centro de la grúa)
            'pos': [10.0, 0.0, 5.0],       # [X, Y, Z] (Z=8.0 es la altura)

            # 2. RANGO DE DISTANCIA (Ignorar objetos muy cerca o muy lejos)
            'dist_min': 0.1,  # Metros (Zona ciega)
            'dist_max': 15.0, # Metros (Alcance máximo útil)
            
            # 3. APERTURA DEL HAZ (El "Cono" de visión)
            # Reduce esto si el sensor ve cosas a los lados que no quieres (pilares, otro carril)
            'ang_min': -95.0, # Grados (Límite Izquierdo)
            'ang_max': 0,  # Grados (Límite Derecho)
            
            # 4. ROTACIÓN DEL MONTAJE (Cómo está instalado el sensor)
            # Pitch: Inclinación arriba/abajo (90 suele ser mirando directo abajo)
            # Yaw: Giro izquierda/derecha (como una brújula)
            # Roll: Giro sobre su propio eje (rara vez se usa)
            'pitch': np.radians(-90), 
            'yaw': np.radians(0), 
            'roll': np.radians(90) 
        }

        # --- SENSOR 2: ESTRUCTURA/PERFIL (IP .101 aprox) ---
        # Este sensor mira de lado o transversal para medir el ancho.
        self.CFG_ESTRUC = {
            'pos': [-5.8, 0.0, 8.0],       # [X, Y, Z]
            'dist_min': 0.1, 
            'dist_max': 15.0,
            
            'ang_min': -45.0, # Grados
            'ang_max': 45.0,  # Grados
            
            # Nota el Yaw en 90: Significa que está girado mirando "de lado"
            'pitch': np.radians(-90), 
            'yaw': np.radians(0), 
            'roll': np.radians(0)
        }

        # ==============================================================================
        # SECCIÓN 3: PARAMETROS DE LÓGICA (PLC)
        # ==============================================================================
        self.TIEMPO_CONFIRMACION = 3.0   # Segundos que el camión debe estar quieto para dar VERDE
        self.PUNTOS_MINIMOS = 30         # Cuantos rebotes láser mínimo para decir "Ahí hay algo"
        
        # Dimensiones mínimas para considerar que es un camión y no una moto/persona
        self.ANCHO_MIN_CAMION = 2.0      # Metros
        self.ALTO_MIN_CAMION = 1.5       # Metros

        # ==========================================
        # CONEXIÓN ROS (No tocar a menos que cambien los Topics)
        # ==========================================
        self.create_subscription(String, 'grua/estado_completo', self.callback_grua_api, 10)
        self.create_subscription(LaserScan, '/scan_distancia', self.callback_longitudinal, 10)
        self.create_subscription(LaserScan, '/scan_estructura', self.callback_estructura, 10)

        # Variables internas (Memoria del sistema)
        self.es_camion_valido = False  
        self.esta_en_posicion = False  
        self.estado_sistema = "ESPERANDO"
        self.timer_inicio = None
        
        # Buffers gráficos
        self.puntos_long = np.zeros((0, 3))
        self.puntos_estruc = np.zeros((0, 3))
        self.new_data = False
        self.actualizar_caja = False

        # ==========================================
        # INTERFAZ GRÁFICA 3D
        # ==========================================
        print(">>> VISUALIZADOR STS - MODIFICABLE <<<")
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name="STS - HMI Operador", width=1280, height=720)
        self.vis.get_render_option().background_color = np.asarray([0.1, 0.1, 0.1]) # Color gris oscuro
        self.vis.get_render_option().point_size = 3.0

        # Crear geometrías vacías
        self.pcd_long = o3d.geometry.PointCloud()
        self.pcd_estruc = o3d.geometry.PointCloud()

        # Crear Caja Objetivo (Se actualiza sola según 20 o 40 pies)
        self.box_target = o3d.geometry.AxisAlignedBoundingBox(
            min_bound=np.array(self.zona_target['min']),
            max_bound=np.array(self.zona_target['max'])
        )
        self.box_target.color = [1, 0, 0] # Rojo al inicio

        # Crear Caja Perfil (Azul fija)
        self.box_profile = o3d.geometry.AxisAlignedBoundingBox(
            min_bound=np.array(self.ZONA_PERFIL['min']),
            max_bound=np.array(self.ZONA_PERFIL['max'])
        )
        self.box_profile.color = [0, 0, 1] 

        # Agregar elementos a la pantalla
        self.vis.add_geometry(self.box_target)
        self.vis.add_geometry(self.box_profile)
        self.vis.add_geometry(self.crear_suelo()) 
        self.vis.add_geometry(self.pcd_long)
        self.vis.add_geometry(self.pcd_estruc)
        
        # Esferas que representan los sensores (Para saber dónde están configurados)
        s1 = o3d.geometry.TriangleMesh.create_sphere(radius=0.2)
        s1.paint_uniform_color([1, 0.5, 0]); s1.translate(self.CFG_LONG['pos'])
        self.vis.add_geometry(s1)

        s2 = o3d.geometry.TriangleMesh.create_sphere(radius=0.2)
        s2.paint_uniform_color([0, 1, 1]); s2.translate(self.CFG_ESTRUC['pos'])
        self.vis.add_geometry(s2)

        self.configurar_camara()
        self.running = True

    # ==========================================
    # LÓGICA DE SENSORES (PROCESAMIENTO)
    # ==========================================
    def callback_longitudinal(self, msg):
        # Transforma datos crudos a XYZ
        points = self.laser_to_xyz(msg, self.CFG_LONG)
        self.puntos_long = points
        
        # Obtenemos los límites de la caja objetivo (Verde/Roja)
        min_b = np.array(self.zona_target['min'])
        max_b = np.array(self.zona_target['max'])

        puntos_validos = 0

        # --- LÓGICA DISCRIMINADA POR MODO ---
        if self.modo_operacion == "DESCARGA":
            # MODO CLÁSICO: Contar puntos dentro de la caja (Pared sólida del contenedor)
            mask = np.all((points >= min_b) & (points <= max_b), axis=1)
            puntos_validos = np.sum(mask)

        elif self.modo_operacion == "CARGA":
            # MODO CHASIS: Filtrar "Fierros Flotantes"
            # 1. Filtramos X e Y (Largo y Ancho) normalmente dentro de la caja
            mask_xy = (points[:,0] >= min_b[0]) & (points[:,0] <= max_b[0]) & \
                      (points[:,1] >= min_b[1]) & (points[:,1] <= max_b[1])
            
            puntos_zona = points[mask_xy]
            
            # 2. FILTRO CRÍTICO DE ALTURA (Z)
            # Ignoramos el suelo (Z < 0.6) y ignoramos la cabina/postes altos (Z > 1.5)
            # Solo queremos ver las vigas del chasis.
            mask_z = (puntos_zona[:,2] > 0.6) & (puntos_zona[:,2] < 1.5)
            
            puntos_validos = np.sum(mask_z)

        # Decisión final (Usamos un umbral un poco más bajo para chasis porque es más delgado)
        umbral = self.PUNTOS_MINIMOS if self.modo_operacion == "DESCARGA" else (self.PUNTOS_MINIMOS / 2)
        
        if puntos_validos > umbral: 
            self.esta_en_posicion = True
        else: 
            self.esta_en_posicion = False
        
        self.evaluar_logica_general()
        self.new_data = True

    def callback_estructura(self, msg):
        points = self.laser_to_xyz(msg, self.CFG_ESTRUC)
        self.puntos_estruc = points

        # Comprobar dimensiones del objeto en la zona PERFIL
        min_b = np.array(self.ZONA_PERFIL['min'])
        max_b = np.array(self.ZONA_PERFIL['max'])
        mask = np.all((points >= min_b) & (points <= max_b), axis=1)
        objeto_puntos = points[mask]

        if len(objeto_puntos) > 20:
            # Calculamos ancho (Y) y alto (Z)
            ancho = np.max(objeto_puntos[:, 1]) - np.min(objeto_puntos[:, 1])
            alto  = np.max(objeto_puntos[:, 2]) - np.min(objeto_puntos[:, 2])
            
            # --- LÓGICA DISCRIMINADA POR MODO ---
            if self.modo_operacion == "DESCARGA":
                # MODO CLÁSICO: Buscamos un contenedor grande y alto
                if ancho > self.ANCHO_MIN_CAMION and alto > self.ALTO_MIN_CAMION: 
                    self.es_camion_valido = True
                else: 
                    self.es_camion_valido = False
            
            elif self.modo_operacion == "CARGA":
                # MODO CHASIS: Buscamos algo ancho pero BAJO (el esqueleto)
                # Debe medir entre 0.8m y 1.6m de alto (aprox)
                condicion_ancho = ancho > self.CHASIS_VACIO['ancho_min']
                condicion_alto  = (alto > self.CHASIS_VACIO['alto_min']) and \
                                  (alto < self.CHASIS_VACIO['alto_max'])
                
                if condicion_ancho and condicion_alto:
                    self.es_camion_valido = True
                else:
                    self.es_camion_valido = False
        else:
            self.es_camion_valido = False

        self.evaluar_logica_general()
        self.new_data = True

    def callback_grua_api(self, msg):
        # Recibe datos de la grúa y decide el MODO DE OPERACIÓN
        try:
            data = json.loads(msg.data)
            
            # 1. Leer datos de la base de datos simulada
            size_raw = str(data.get("spreaderSize", 40))
            twistlock_cerrado = data.get("twistlocksClosed", False) # True o False
            trolley_pos = data.get("trolleyPosition", "TIERRA")     # "MAR" o "TIERRA"

            # 2. Determinar MODO DE OPERACIÓN (Lógica de negocio)
            # Si trae carga del mar (Twistlock cerrado + Mar) -> Viene a CARGAR al camión (Busca chasis vacío)
            # NOTA: Ajusta esta lógica si tu proceso es diferente.
            if twistlock_cerrado and trolley_pos == "MAR":
                nuevo_modo_op = "CARGA" 
            else:
                # Si twistlocks están abiertos o el trolley está en tierra esperando descargar -> Busca Contenedor
                nuevo_modo_op = "DESCARGA" 

            # 3. Detectar cambios y reiniciar si es necesario
            cambio_tamaño = (size_raw in self.DB_GEOMETRIA and size_raw != self.modo_actual)
            cambio_modo   = (nuevo_modo_op != self.modo_operacion)

            if cambio_tamaño or cambio_modo:
                print(f">>> CAMBIO: Modo {nuevo_modo_op} | Tamaño {size_raw} Pies <<<")
                
                # Actualizar variables
                self.modo_actual = size_raw
                self.zona_target = self.DB_GEOMETRIA[size_raw]
                self.modo_operacion = nuevo_modo_op
                
                # Actualizar gráficos y reiniciar semáforo
                self.actualizar_caja = True
                self.estado_sistema = "ESPERANDO"
                self.timer_inicio = None

        except: pass

    # ==========================================
    # CEREBRO DEL SISTEMA (SEMÁFORO)
    # ==========================================
    def evaluar_logica_general(self):
        # CONDICIÓN AND: Debe ser válido (ancho) Y estar en posición (longitudinal)
        condicion_segura = self.es_camion_valido and self.esta_en_posicion

        if condicion_segura:
            if self.estado_sistema == "ESPERANDO":
                self.estado_sistema = "VALIDANDO"
                self.timer_inicio = time.time() # Empezar cronómetro
                self.box_target.color = [1, 1, 0] # AMARILLO

            elif self.estado_sistema == "VALIDANDO":
                # Chequear si pasó el tiempo
                if (time.time() - self.timer_inicio) >= self.TIEMPO_CONFIRMACION:
                    self.estado_sistema = "CONFIRMADO"
                    print(f">>> [OK] CAMIÓN {self.modo_actual} PIES LISTO <<<")
                    self.box_target.color = [0, 1, 0] # VERDE

        else:
            # Si se mueve o desaparece, volver a rojo
            if self.estado_sistema == "CONFIRMADO": 
                print(">>> CAMIÓN SALIENDO... <<<")
            self.estado_sistema = "ESPERANDO"
            self.timer_inicio = None
            self.box_target.color = [1, 0, 0] # ROJO

    # ==========================================
    # FUNCIONES MATEMÁTICAS (NO NECESITAS TOCAR ESTO)
    # ==========================================
    def laser_to_xyz(self, msg, cfg):
        # Esta función convierte coordenadas polares del sensor a cartesianas globales
        # Respetando los límites de ángulo que pusiste arriba.
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        ranges = np.array(msg.ranges)
        min_len = min(len(angles), len(ranges))
        angles = angles[:min_len]; ranges = ranges[:min_len]

        # Convertir configuración (grados) a matemáticas (radianes)
        lim_min_rad = np.radians(cfg['ang_min'])
        lim_max_rad = np.radians(cfg['ang_max'])

        # Filtro de distancia y ángulo
        valid = (ranges > 0.1) & \
                (ranges > cfg['dist_min']) & (ranges < cfg['dist_max']) & \
                (angles >= lim_min_rad) & (angles <= lim_max_rad)

        r = ranges[valid]
        a = angles[valid]
        
        if len(r) == 0: return np.zeros((0, 3))

        x = r * np.cos(a)
        y = r * np.sin(a)
        z = np.zeros_like(x)

        # Aplicar rotaciones
        if cfg['roll'] != 0:
            c, s = np.cos(cfg['roll']), np.sin(cfg['roll'])
            y_n = y * c - z * s; z_n = y * s + z * c; y, z = y_n, z_n
        if cfg['pitch'] != 0:
            c, s = np.cos(cfg['pitch']), np.sin(cfg['pitch'])
            x_n = x * c - z * s; z_n = x * s + z * c; x, z = x_n, z_n
        if cfg['yaw'] != 0:
            c, s = np.cos(cfg['yaw']), np.sin(cfg['yaw'])
            x_n = x * c - y * s; y_n = x * s + y * c; x, y = x_n, y_n

        # Aplicar posición
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
    gui = VisualizadorSTS()
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
