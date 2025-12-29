import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String  # 🔥 Necesario para recibir el JSON de la grúa
import open3d as o3d
import numpy as np
import threading
import time
import json # 🔥 Necesario para leer los datos

class VisualizadorSTS(Node):
    def __init__(self):
        super().__init__('visualizador_sts_realtime')
        
        # ==========================================
        # 1. CONFIGURACIÓN DE GEOMETRÍA (CAJAS VIRTUALES)
        # ==========================================
        # Aquí definimos dónde debe estacionarse el camión según el Spreader
        self.DB_GEOMETRIA = {
            '40': { # CASO SPREADER 40 PIES
                'min': [-6.0, -1.2, 1.0], 
                'max': [ 6.0,  1.2, 1.6], 
                'desc': "40 PIES (ESTÁNDAR)" 
            },
            '20': { # CASO SPREADER 20 PIES (Cola del chasis)
                'min': [-6.0, -1.2, 1.0], 
                'max': [ 0.0,  1.2, 1.6], 
                'desc': "20 PIES (POSICIÓN TRASERA)" 
            }
        }
        
        # Estado inicial (Asumimos 40 hasta que la grúa diga lo contrario)
        self.modo_actual = '40'
        self.zona_actual = self.DB_GEOMETRIA['40']
        
        # Variables de la Grúa (Para mostrar en pantalla/consola)
        self.grua_trolley = "N/A"
        self.grua_twistlock = "N/A"

        # ==========================================
        # 2. CONEXIÓN CON LA GRÚA (Suscripción ROS) 🔥
        # ==========================================
        # Nos "colgamos" del cable que tira el client_API.py
        self.create_subscription(String, 'grua/estado_completo', self.callback_grua_api, 10)
        print(">>> SISTEMA INICIADO: ESPERANDO DATOS DE LA GRÚA... <<<")

        # --- CONFIG SENSOR LÁSER (TU HARDWARE) ---
        self.CFG_SENSOR = {
            'pos': [0.0, 0.0, 3.0],      # Altura de la grúa
            'dist_min': 0.1, 'dist_max': 15.0,
            'ang_min': -15.0, 'ang_max': 15.0,
            'pitch': np.radians(-90), 'yaw': np.radians(90), 'roll': 0
        }

        # --- LÓGICA DE SEGURIDAD (PLC) ---
        self.timer_inicio = None
        self.TIEMPO_CONFIRMACION = 3.0 
        self.LONGITUD_MINIMA = 3.5    
        self.estado_validacion = "VACIO" 

        # ROS Setup Láser
        self.create_subscription(LaserScan, '/scan_estructura', self.callback_sensor_laser, 10)
        self.pcd_buffer = o3d.geometry.PointCloud()
        self.new_data = False
        self.actualizar_caja = False # Bandera para actualizar gráfico

        # --- INTERFAZ GRÁFICA ---
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name="STS - Operación en Vivo", width=1280, height=720)
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

    # =========================================================
    # 🔥 CALLBACK 1: CEREBRO DE LA GRÚA (Recibe datos API)
    # =========================================================
    def callback_grua_api(self, msg):
        try:
            # 1. Decodificar el mensaje que viene de client_API.py
            data = json.loads(msg.data)
            
            # 2. Extraer datos (usando .get para seguridad)
            size_raw = data.get("spreaderSize", 40) # Si falla, asume 40
            self.grua_trolley = data.get("trolleyPos", "N/A")
            self.grua_twistlock = data.get("spreaderTwistlock", "N/A")

            # 3. Lógica de selección de Caja (20 vs 40)
            # Convertimos a string para comparar con nuestras llaves '20' o '40'
            modo_nuevo = str(size_raw)
            
            # Solo si el spreader cambió de tamaño, actualizamos la caja
            if modo_nuevo in self.DB_GEOMETRIA and modo_nuevo != self.modo_actual:
                print(f"\n>>> [GRÚA] CAMBIO DE SPREADER DETECTADO: {modo_nuevo} PIES <<<")
                self.modo_actual = modo_nuevo
                self.zona_actual = self.DB_GEOMETRIA[modo_nuevo]
                self.actualizar_caja = True # Avisamos al loop gráfico
                
                # Reseteamos validación porque cambió la condición
                self.estado_validacion = "VACIO"
                self.timer_inicio = None
                self.box_visual.color = [1, 0, 0]

        except json.JSONDecodeError:
            print("Error recibiendo datos de la grúa")

    # =========================================================
    # 🔥 CALLBACK 2: OJOS DEL SISTEMA (Recibe Láser)
    # =========================================================
    def callback_sensor_laser(self, msg):
        try:
            points = self.laser_to_xyz(msg)
            es_camion, largo = self.validar_presencia(points)

            # Lógica de Semáforo (Igual que antes)
            if es_camion and largo > self.LONGITUD_MINIMA:
                if self.estado_validacion == "VACIO":
                    self.estado_validacion = "VALIDANDO"
                    self.timer_inicio = time.time()
                    print(f"... Detectando chasis ({largo:.1f}m) ... Spreader: {self.modo_actual}' ...")
                    self.box_visual.color = [1, 1, 0] # AMARILLO

                elif self.estado_validacion == "VALIDANDO":
                    if (time.time() - self.timer_inicio) >= self.TIEMPO_CONFIRMACION:
                        self.estado_validacion = "CONFIRMADO"
                        print(f">>> [OK] CAMIÓN POSICIONADO PARA {self.modo_actual} PIES <<<")
                        print(f"    (Trolley: {self.grua_trolley} | Candados: {self.grua_twistlock})")
                        self.box_visual.color = [0, 1, 0] # VERDE

                elif self.estado_validacion == "CONFIRMADO":
                    self.box_visual.color = [0, 1, 0]
            else:
                if self.estado_validacion == "CONFIRMADO":
                    print(">>> CAMIÓN SALIENDO... <<<")
                self.estado_validacion = "VACIO"
                self.timer_inicio = None
                self.box_visual.color = [1, 0, 0]

            self.pcd_buffer.points = o3d.utility.Vector3dVector(points)
            self.pcd_buffer.paint_uniform_color([0, 1, 1])
            self.new_data = True
        except Exception: pass

    # --- FUNCIONES MATEMÁTICAS (NO TOCAR) ---
    def laser_to_xyz(self, msg):
        cfg = self.CFG_SENSOR
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        ranges = np.array(msg.ranges)
        min_len = min(len(angles), len(ranges))
        angles = angles[:min_len]; ranges = ranges[:min_len]

        valid = (ranges > 0.1) & (ranges > cfg['dist_min']) & (ranges < cfg['dist_max'])
        r = ranges[valid]; a = angles[valid]
        if len(r) == 0: return np.zeros((0, 3))

        x = r * np.cos(a); y = r * np.sin(a); z = np.zeros_like(x)

        if cfg['roll']!=0: c,s=np.cos(cfg['roll']),np.sin(cfg['roll']); y_n=y*c-z*s; z_n=y*s+z*c; y=y_n; z=z_n
        if cfg['pitch']!=0: c,s=np.cos(cfg['pitch']),np.sin(cfg['pitch']); x_n=x*c-z*s; z_n=x*s+z*c; x=x_n; z=z_n
        if cfg['yaw']!=0: c,s=np.cos(cfg['yaw']),np.sin(cfg['yaw']); x_n=x*c-y*s; y_n=x*s+y*c; x=x_n; y=y_n
        return np.vstack((x + cfg['pos'][0], y + cfg['pos'][1], z + cfg['pos'][2])).T

    def validar_presencia(self, points):
        min_b = np.array(self.zona_actual['min'])
        max_b = np.array(self.zona_actual['max'])
        mask = (points[:, 0] >= min_b[0]) & (points[:, 0] <= max_b[0]) & \
               (points[:, 1] >= min_b[1]) & (points[:, 1] <= max_b[1]) & \
               (points[:, 2] >= min_b[2]) & (points[:, 2] <= max_b[2])
        puntos_dentro = points[mask]
        if len(puntos_dentro) < 20: return False, 0.0
        return True, (np.max(puntos_dentro[:, 0]) - np.min(puntos_dentro[:, 0]))

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
    t = threading.Thread(target=rclpy.spin, args=(gui,), daemon=True)
    t.start()

    try:
        while gui.running:
            # 1. Si llegaron datos del LÁSER, actualizar nube
            if gui.new_data:
                gui.vis.update_geometry(gui.pcd_buffer)
                gui.vis.update_geometry(gui.box_visual) # Mantener color actualizado
                gui.new_data = False
            
            # 2. Si llegaron datos de la GRÚA que requieren cambiar la caja
            if gui.actualizar_caja:
                gui.box_visual.min_bound = np.array(gui.zona_actual['min'])
                gui.box_visual.max_bound = np.array(gui.zona_actual['max'])
                gui.vis.update_geometry(gui.box_visual)
                gui.actualizar_caja = False
            
            gui.vis.poll_events()
            gui.vis.update_renderer()
            time.sleep(0.01)
    except KeyboardInterrupt: pass
    finally: gui.vis.destroy_window(); rclpy.shutdown()

if __name__ == '__main__': main()
