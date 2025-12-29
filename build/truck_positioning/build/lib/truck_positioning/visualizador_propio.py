import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import open3d as o3d
import numpy as np
import threading
import time
import json
import os

class VisualizadorSTS(Node):
    def __init__(self):
        super().__init__('visualizador_sts_automatico')
        
        # ==========================================
        # 1. CONFIGURACIÓN DE AUTOMATIZACIÓN (BASE DE DATOS)
        # ==========================================
        self.archivo_db = "base_datos_camiones.json"
        self.cola_trabajo = [] # Aquí cargaremos la lista
        self.camion_actual = None
        
        # Cargar la base de datos al iniciar
        self.cargar_base_datos()

        # --- PRESETS (Geometría) ---
        self.DB_GEOMETRIA = {
            '40': { 'min': [-6.0, -1.2, 1.0], 'max': [ 6.0,  1.2, 1.6], 'desc': "40 PIES (ESTÁNDAR)" },
            '20': { 'min': [-6.0, -1.2, 1.0], 'max': [ 0.0,  1.2, 1.6], 'desc': "20 PIES (COLA)" }
        }
        
        # Configuramos el primer modo según la base de datos
        if self.cola_trabajo:
            self.modo_actual = self.cola_trabajo[0]['tipo']
            self.camion_actual = self.cola_trabajo[0]
            print(f">>> ESPERANDO PRIMER CAMIÓN: {self.camion_actual['patente']} (Tipo {self.modo_actual}) <<<")
        else:
            self.modo_actual = '40' # Default si no hay DB
            print(">>> ALERTA: Base de datos vacía. Usando modo Manual 40 pies. <<<")

        self.zona_actual = self.DB_GEOMETRIA[self.modo_actual]

        # --- CONFIG SENSOR ---
        self.CFG_SENSOR = {
            'pos': [0.0, 0.0, 8.0],
            'dist_min': 0.1, 'dist_max': 15.0,
            'ang_min': -45.0, 'ang_max': 45.0,
            'pitch': np.radians(90), 'yaw': 0, 'roll': 0
        }

        # --- VARIABLES DE SEGURIDAD (PLC) ---
        self.timer_inicio = None
        self.TIEMPO_CONFIRMACION = 3.0 
        self.LONGITUD_MINIMA = 3.5    
        self.estado_validacion = "VACIO" # Estados: VACIO -> VALIDANDO -> CONFIRMADO

        # ROS Setup
        self.create_subscription(LaserScan, '/scan_estructura', self.callback_sensor, 10)
        self.pcd_buffer = o3d.geometry.PointCloud()
        self.new_data = False

        # --- INTERFAZ GRÁFICA ---
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name="STS - Sistema Automatizado por DB", width=1280, height=720)
        self.vis.get_render_option().background_color = np.asarray([0.1, 0.1, 0.1])
        self.vis.get_render_option().point_size = 4.0

        # Objetos Visuales
        self.box_visual = o3d.geometry.AxisAlignedBoundingBox(
            min_bound=np.array(self.zona_actual['min']),
            max_bound=np.array(self.zona_actual['max'])
        )
        self.box_visual.color = [1, 0, 0] # Rojo
        self.vis.add_geometry(self.box_visual)
        self.vis.add_geometry(self.crear_suelo())
        
        s1 = o3d.geometry.TriangleMesh.create_sphere(radius=0.3)
        s1.paint_uniform_color([0, 1, 1]); s1.translate(self.CFG_SENSOR['pos'])
        self.vis.add_geometry(s1)
        self.vis.add_geometry(self.pcd_buffer)

        self.configurar_camara()
        self.running = True

    # --- FUNCIONES DE BASE DE DATOS ---
    def cargar_base_datos(self):
        try:
            # Buscamos el archivo en la misma carpeta que el script
            ruta = os.path.join(os.path.dirname(__file__), self.archivo_db)
            if os.path.exists(ruta):
                with open(ruta, 'r') as f:
                    self.cola_trabajo = json.load(f)
                print(f"\n>>> BASE DE DATOS CONECTADA: {len(self.cola_trabajo)} camiones en cola. <<<")
            else:
                print(f"\n>>> ERROR: No encontré el archivo '{self.archivo_db}'. Creando uno de prueba... <<<")
                # Crear archivo dummy si no existe para que no falle
                datos_prueba = [
                    {"id": 1, "tipo": "40", "patente": "TEST-40"},
                    {"id": 2, "tipo": "20", "patente": "TEST-20"},
                    {"id": 3, "tipo": "40", "patente": "TEST-40B"}
                ]
                with open(ruta, 'w') as f:
                    json.dump(datos_prueba, f)
                self.cola_trabajo = datos_prueba
        except Exception as e:
            print(f"Error leyendo DB: {e}")
            self.cola_trabajo = []

    def avanzar_siguiente_camion(self):
        # Esta función se llama cuando el camión anterior SE VA (Vuelve a rojo)
        if len(self.cola_trabajo) > 0:
            # Sacamos el primero de la lista (ya fue atendido)
            completado = self.cola_trabajo.pop(0)
            print(f">>> FINALIZADO: Camión {completado['patente']} despachado. <<<")
            
            # Preparamos el siguiente
            if len(self.cola_trabajo) > 0:
                self.camion_actual = self.cola_trabajo[0]
                nuevo_modo = self.camion_actual['tipo']
                
                # Actualizamos la geometría automáticamente
                self.modo_actual = nuevo_modo
                self.zona_actual = self.DB_GEOMETRIA[nuevo_modo]
                
                # Actualizamos gráficos
                self.box_visual.min_bound = np.array(self.zona_actual['min'])
                self.box_visual.max_bound = np.array(self.zona_actual['max'])
                self.vis.update_geometry(self.box_visual)
                
                print(f"\n>>> SIGUIENTE TURNO: Patente {self.camion_actual['patente']} (Tipo {nuevo_modo}) <<<")
                print(">>> Ajustando caja virtual... LISTO. Esperando llegada... <<<")
            else:
                print("\n>>> COLA DE TRABAJO VACÍA. Esperando actualización de DB... <<<")
                self.box_visual.color = [0.5, 0.5, 0.5] # Gris (Inactivo)
                self.vis.update_geometry(self.box_visual)

    def laser_to_xyz(self, msg):
        cfg = self.CFG_SENSOR
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        ranges = np.array(msg.ranges)
        min_len = min(len(angles), len(ranges))
        angles = angles[:min_len]; ranges = ranges[:min_len]

        lim_min_rad = np.radians(cfg['ang_min'])
        lim_max_rad = np.radians(cfg['ang_max'])
        valid = (ranges > 0.1) & (ranges > cfg['dist_min']) & (ranges < cfg['dist_max']) & \
                (angles > lim_min_rad) & (angles < lim_max_rad)
        
        r = ranges[valid]; a = angles[valid]
        if len(r) == 0: return np.zeros((0, 3))

        x = r * np.cos(a); y = r * np.sin(a); z = np.zeros_like(x)

        # Rotaciones
        if cfg['roll'] != 0:
            c, s = np.cos(cfg['roll']), np.sin(cfg['roll'])
            y_n = y*c - z*s; z_n = y*s + z*c; y=y_n; z=z_n
        if cfg['pitch'] != 0:
            c, s = np.cos(cfg['pitch']), np.sin(cfg['pitch'])
            x_n = x*c - z*s; z_n = x*s + z*c; x=x_n; z=z_n
        if cfg['yaw'] != 0:
            c, s = np.cos(cfg['yaw']), np.sin(cfg['yaw'])
            x_n = x*c - y*s; y_n = x*s + y*c; x=x_n; y=y_n

        return np.vstack((x + cfg['pos'][0], y + cfg['pos'][1], z + cfg['pos'][2])).T

    def validar_presencia(self, points):
        min_b = np.array(self.zona_actual['min'])
        max_b = np.array(self.zona_actual['max'])
        mask = (points[:, 0] >= min_b[0]) & (points[:, 0] <= max_b[0]) & \
               (points[:, 1] >= min_b[1]) & (points[:, 1] <= max_b[1]) & \
               (points[:, 2] >= min_b[2]) & (points[:, 2] <= max_b[2])
        puntos_dentro = points[mask]

        if len(puntos_dentro) < 20: return False, 0.0
        
        min_x = np.min(puntos_dentro[:, 0])
        max_x = np.max(puntos_dentro[:, 0])
        return True, (max_x - min_x)

    def callback_sensor(self, msg):
        try:
            points = self.laser_to_xyz(msg)
            es_camion, largo = self.validar_presencia(points)

            # --- MÁQUINA DE ESTADOS FINITOS (AUTOMATIZACIÓN) ---
            if es_camion and largo > self.LONGITUD_MINIMA:
                # Evento: Detección válida
                if self.estado_validacion == "VACIO":
                    self.estado_validacion = "VALIDANDO"
                    self.timer_inicio = time.time()
                    print(f"... Detectando {largo:.2f}m ... Validando ...")
                    self.box_visual.color = [1, 1, 0] # AMARILLO

                elif self.estado_validacion == "VALIDANDO":
                    if (time.time() - self.timer_inicio) >= self.TIEMPO_CONFIRMACION:
                        self.estado_validacion = "CONFIRMADO"
                        print(f">>> [OPERANDO] CAMIÓN '{self.camion_actual['patente']}' OK <<<")
                        self.box_visual.color = [0, 1, 0] # VERDE

                elif self.estado_validacion == "CONFIRMADO":
                    # Mantiene verde mientras el camión siga ahí
                    self.box_visual.color = [0, 1, 0]
            
            else:
                # Evento: No hay camión (o se fue)
                if self.estado_validacion == "CONFIRMADO":
                    # EL CAMIÓN SE ACABA DE IR -> PASAR AL SIGUIENTE
                    print(">>> CAMIÓN SALIENDO DE ZONA... <<<")
                    self.avanzar_siguiente_camion()
                
                # Resetear estado
                self.estado_validacion = "VACIO"
                self.timer_inicio = None
                self.box_visual.color = [1, 0, 0] # ROJO (Esperando)

            self.pcd_buffer.points = o3d.utility.Vector3dVector(points)
            self.pcd_buffer.paint_uniform_color([0, 1, 1])
            self.new_data = True
        except Exception as e: pass

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
        ctr.set_lookat([0,0,0]); ctr.set_front([0,-0.8,0.8]); ctr.set_up([0,0,1]); ctr.set_zoom(0.5)

def main(args=None):
    rclpy.init(args=args)
    gui = VisualizadorSTS()
    threading.Thread(target=rclpy.spin, args=(gui,), daemon=True).start()
    try:
        while gui.running:
            if gui.new_data:
                gui.vis.update_geometry(gui.pcd_buffer)
                gui.vis.update_geometry(gui.box_visual)
                gui.new_data = False
            gui.vis.poll_events(); gui.vis.update_renderer(); time.sleep(0.01)
    except KeyboardInterrupt: pass
    finally: gui.vis.destroy_window(); rclpy.shutdown()

if __name__ == '__main__': main()
