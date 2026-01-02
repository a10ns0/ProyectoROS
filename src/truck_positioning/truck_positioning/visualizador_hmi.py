import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Float64 # <--- IMPORT AGREGADO
from tf2_ros import Buffer
from tf2_ros.transform_listener import TransformListener
import open3d as o3d
import numpy as np
import threading
import time

class SistemaLogicaPLC(Node):
    def __init__(self):
        super().__init__('sistema_logica_plc')
        
        # =========================================================
        # 1. ESCALADO (1.5 METROS DE SEPARACIÓN)
        # =========================================================
        DISTANCIA_REAL_ORIGINAL = 20.4
        DISTANCIA_MAQUETA = 1.5 
        self.SCALE = DISTANCIA_MAQUETA / DISTANCIA_REAL_ORIGINAL # aprox 0.0735
        
        # --- UMBRALES DE ALTURA (CALCULADOS) ---
        # MODO CARGA (Camión llega con Contenedor)
        self.H_MIN_CONTENEDOR_VALI = 3.5 * self.SCALE  # 25.7 cm
        self.H_FILTRO_CONTENEDOR   = 3.0 * self.SCALE  # 22.0 cm (Ignorar todo lo bajo)

        # MODO DESCARGA (Camión llega Vacío)
        self.H_MAX_CHASIS_VALI     = 1.8 * self.SCALE  # 13.2 cm
        self.H_FILTRO_CHASIS_HIGH  = 1.75 * self.SCALE # 12.8 cm (Corte superior)
        self.H_FILTRO_CHASIS_LOW   = 1.0 * self.SCALE  # 7.3 cm (Corte inferior/suelo)

        print(f"=== SISTEMA STS LOGIC (Escala {self.SCALE:.4f}) ===")
        print(f"FILTRO CONTENEDOR (CARGA): Solo ve objetos > {self.H_FILTRO_CONTENEDOR*100:.1f} cm")
        print(f"FILTRO CHASIS (DESCARGA): Solo ve objetos entre {self.H_FILTRO_CHASIS_LOW*100:.1f} cm y {self.H_FILTRO_CHASIS_HIGH*100:.1f} cm")

        # =========================================================
        # 2. VARIABLES DE "BASE DE DATOS" (SIMULADAS)
        # =========================================================
        self.db_spreader_size = '40'    # '20' o '40'
        self.db_twistlocks    = 'CLOSE' # 'OPEN' (Cargar al barco) / 'CLOSE' (Descargar del barco)
        # Nota: Trolley no lo simulamos visualmente, asumimos que está en posición si llegan datos.
        
        # Estado Lógico derivado
        self.modo_operacion = 'DESCARGA_DEL_BARCO' # Por defecto si twistlock close

        # =========================================================
        # 3. ZONAS GEOMÉTRICAS
        # =========================================================
        self.DB_GEOMETRIA = {
            '40': { 'min': [-6.0*self.SCALE, -1.2*self.SCALE, 0.0], 'max': [ 6.0*self.SCALE,  1.2*self.SCALE, 5.0*self.SCALE] },
            '20': { 'min': [-3.0*self.SCALE, -1.2*self.SCALE, 0.0], 'max': [ 3.0*self.SCALE,  1.2*self.SCALE, 5.0*self.SCALE] }
        }
        self.zona_target = self.DB_GEOMETRIA['40']
        
        self.posicion_detectada = None
        self.ultimo_debug = 0

        # =========================================================
        # 4. CONFIGURACIÓN SENSORES (MAQUETA 1.5M)
        # =========================================================
        # Altura física sensores: aprox 92cm (12.5m * scale)
        self.Z_SENSOR = 12.5 * self.SCALE
        self.X_SENSOR = 10.2 * self.SCALE # 0.75m

        # SENSOR 2 (Longitudinal)
        self.CFG_LONG = {
            'pos': [10.2 * self.SCALE, 0.0, 12.5 * self.SCALE], 
            'dist_min': 0.05, 
            'dist_max': 10.0,      # <--- AUMENTADO A 10 METROS (Ver paredes lejanas)
            
            # ABANICO COMPLETO (360 grados si es posible)
            'ang_min': -180.0,     # <--- ABIERTO AL MÁXIMO
            'ang_max': 180.0,      # <--- ABIERTO AL MÁXIMO
            
            'pitch': np.radians(0), 
            'yaw': np.radians(180), 
            'roll': np.radians(90)
        }

       # SENSOR 1 (Estructura) - AZUL
        # MODO PÁNICO: Sin recortes, rango enorme, ángulo total.
        self.CFG_ESTRUC = {
            'pos': [-10.2 * self.SCALE, 11.19 * self.SCALE, 12.5 * self.SCALE],
            'dist_min': 0.0,       # <--- ACEPTA TODO (Incluso ruido cerca)
            'dist_max': 100.0,     # <--- ACEPTA TODO (Hasta 100 metros)
            'ang_min': -180.0,     # <--- 360 GRADOS
            'ang_max': 180.0,      # <--- 360 GRADOS
            'pitch': np.radians(0), # Mira al frente (más fácil de ver)
            'yaw': np.radians(180), 
            'roll': np.radians(0)
        }
        # ROS
        self.create_subscription(LaserScan, '/scan_distancia', self.cb_longitudinal, 10)
        self.create_subscription(LaserScan, '/scan_estructura', self.cb_estructura, 10)
        self.pub_cmd = self.create_publisher(String, '/truck_cmd', 10)
        

        # === NUEVO: CANALES PARA PLOTJUGGLER / RQT_PLOT ===
        self.pub_distancia_graph = self.create_publisher(Float64, '/tps/debug/posicion_x', 10)
        self.pub_error_graph     = self.create_publisher(Float64, '/tps/debug/error_m', 10)

        self.puntos_long = np.zeros((0, 3))
        self.puntos_estruc = np.zeros((0, 3))
        self.new_data = False
        self.init_gui()
        self.running = True

    def init_gui(self):
        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.vis.create_window("CONTROL LÓGICO STS", 1280, 720)
        self.vis.get_render_option().point_size = 5.0

        # --- SIMULACIÓN DE BASE DE DATOS (TECLADO) ---
        self.vis.register_key_callback(ord("T"), self.toggle_twistlock)
        self.vis.register_key_callback(ord("S"), self.toggle_spreader)
        
        # --- VISUALIZACIÓN ---
        self.pcd_long = o3d.geometry.PointCloud()
        self.pcd_estruc = o3d.geometry.PointCloud()

        self.box_target = o3d.geometry.AxisAlignedBoundingBox(
            min_bound=np.array(self.zona_target['min']), max_bound=np.array(self.zona_target['max']))
        self.box_target.color = [1, 0, 0]

        # PLANOS DE REFERENCIA (ALTURAS DE CORTE)
        # Dibujamos líneas para que sepas qué altura es 1.7m y 3.5m en tu maqueta
        self.vis.add_geometry(self.crear_referencia_altura(self.H_FILTRO_CHASIS_HIGH, [1, 1, 0])) # Amarillo: Límite Chasis
        self.vis.add_geometry(self.crear_referencia_altura(self.H_FILTRO_CONTENEDOR, [0, 1, 1]))  # Cyan: Límite Contenedor

        self.vis.add_geometry(self.box_target)
        self.vis.add_geometry(self.crear_suelo_escalado()) 
        self.vis.add_geometry(self.pcd_long)
        self.vis.add_geometry(self.pcd_estruc)
        
        # Sensores
        s1 = o3d.geometry.TriangleMesh.create_sphere(radius=0.3 * self.SCALE)
        s1.translate(self.CFG_LONG['pos']); s1.paint_uniform_color([1, 0.5, 0])
        self.vis.add_geometry(s1)

        print("\n--- CONTROLES DB SIMULADA ---")
        print("[T] Alternar Twistlocks (OPEN=Carga / CLOSE=Descarga)")
        print("[S] Alternar Spreader (20 / 40 Pies)")
        
        ctr = self.vis.get_view_control()
        ctr.set_lookat([0,0,0]); ctr.set_front([0.0, -0.5, 0.8]); ctr.set_zoom(0.25)

    # ==========================================
    # LÓGICA DE NEGOCIO (EL CORAZÓN)
    # ==========================================
    def toggle_twistlock(self, vis):
        if self.db_twistlocks == 'CLOSE':
            self.db_twistlocks = 'OPEN'
            self.modo_operacion = 'CARGA_AL_BARCO' # Camión trae Contenedor
        else:
            self.db_twistlocks = 'CLOSE'
            self.modo_operacion = 'DESCARGA_DEL_BARCO' # Camión viene Vacío
        
        print(f">>> DB UPDATE: Twistlocks {self.db_twistlocks} -> Modo {self.modo_operacion}")
        return False

    def toggle_spreader(self, vis):
        self.db_spreader_size = '40' if self.db_spreader_size == '20' else '20'
        
        # Actualizar Caja Visual
        self.zona_target = self.DB_GEOMETRIA[self.db_spreader_size]
        self.box_target.min_bound = np.array(self.zona_target['min'])
        self.box_target.max_bound = np.array(self.zona_target['max'])
        
        print(f">>> DB UPDATE: Spreader {self.db_spreader_size} Pies")
        # Forzamos actualización visual inmediata
        self.vis.update_geometry(self.box_target)
        return False

    def cb_longitudinal(self, msg):
        points = self.laser_to_xyz(msg, self.CFG_LONG)
        self.puntos_long = points
        
        # 1. Filtro Ancho Calle (Común para todos)
        ancho_calle = 3.5 * self.SCALE
        mask_calle = (points[:,1] > -ancho_calle) & (points[:,1] < ancho_calle)
        
        # 2. FILTRO DINÁMICO SEGÚN TWISTLOCKS
        mask_altura = None

        if self.modo_operacion == 'CARGA_AL_BARCO':
            # Twistlock OPEN -> Esperamos Camión con Contenedor
            # REGLA: Ignorar todo lo que mida menos de 3 metros reales (22cm)
            mask_altura = (points[:,2] > self.H_FILTRO_CONTENEDOR)
        
        elif self.modo_operacion == 'DESCARGA_DEL_BARCO':
            # Twistlock CLOSE -> Esperamos Chasis Vacío
            # REGLA: Ignorar suelo (< 1m) Y ignorar cabina (> 1.75m)
            mask_altura = (points[:,2] > self.H_FILTRO_CHASIS_LOW) & \
                          (points[:,2] < self.H_FILTRO_CHASIS_HIGH)

        # Aplicar Filtros
        pts_validos = points[mask_calle & mask_altura]

        if len(pts_validos) > 5:
            self.posicion_detectada = np.mean(pts_validos[:,0])
        else:
            self.posicion_detectada = None

        self.evaluar_semaforo()
        self.new_data = True

    def cb_estructura(self, msg):
        # 1. Obtener Puntos CRUDOS (Sin filtrar ángulo ni distancia aquí primero)
        # Queremos saber cuántos llegan realmente
        ranges = np.array(msg.ranges)
        total_puntos = len(ranges)
        puntos_validos_sensor = np.sum((ranges > 0.05) & (ranges < 40.0))
        
        # 2. Procesar con nuestra función visual
        points = self.laser_to_xyz(msg, self.CFG_ESTRUC)
        self.puntos_estruc = points
        self.new_data = True
        
        # 3. REPORTE DE DIAGNÓSTICO (Solo imprime cada 1 seg para no saturar)
        if time.time() - self.ultimo_debug > 1.0:
            print(f"--- DIAGNOSTICO SENSOR AZUL ---")
            print(f"   Datos Crudos recibidos: {total_puntos}")
            print(f"   Datos con rango válido: {puntos_validos_sensor}")
            print(f"   Puntos DIBUJADOS (después de filtros): {len(points)}")
            
            if len(points) == 0:
                print("   ¡ALERTA! El sensor envía datos pero TU CÓDIGO LOS BORRA TODOS.")
                print("   Revisa: dist_min/max o ang_min/max en CFG_ESTRUC")
            print("-------------------------------")





    def evaluar_semaforo(self):
            if self.posicion_detectada is None:
                self.box_target.color = [1, 0, 0] 
                return

            # 1. Calculamos el centro de la meta actual
            centro_meta_x = (self.zona_target['min'][0] + self.zona_target['max'][0]) / 2.0
            
            # 2. Calculamos el error CON SIGNO (para saber si está adelante o atrás en la gráfica)
            error_con_signo = self.posicion_detectada - centro_meta_x
            error_absoluto = abs(error_con_signo)
            
            # === NUEVO: PUBLICAR PARA GRAFICA ===
            # Enviamos la posición actual X
            msg_pos = Float64()
            msg_pos.data = float(self.posicion_detectada)
            self.pub_distancia_graph.publish(msg_pos)

            # Enviamos el error (Asi ves en la gráfica como se acerca a 0)
            msg_err = Float64()
            msg_err.data = float(error_con_signo)
            self.pub_error_graph.publish(msg_err)
            # ====================================

            # Feedback en Terminal (Tu código original)
            if time.time() - self.ultimo_debug > 1.0:
                status_str = f"Modo: {self.modo_operacion} ({self.db_spreader_size}')"
                print(f"[{status_str}] Obj: {self.posicion_detectada:.3f}m | Target: {centro_meta_x:.3f}m | Error: {error_con_signo:.3f}m")
                self.ultimo_debug = time.time()

            # Lógica de colores (Usa el absoluto)
            TOLERANCIA = 0.5 * self.SCALE 
            if error_absoluto <= TOLERANCIA:
                self.box_target.color = [0, 1, 0] # VERDE
            elif error_absoluto <= (TOLERANCIA * 4):
                self.box_target.color = [1, 1, 0] # AMARILLO
            else:
                self.box_target.color = [1, 0, 0] # ROJO

    # ==========================================
    # UTILIDADES MATEMÁTICAS
    # ==========================================
    def laser_to_xyz(self, msg, cfg):
        # 1. Filtro de Ángulo ESTRICTO
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        ranges = np.array(msg.ranges)
        min_len = min(len(angles), len(ranges))
        angles = angles[:min_len]; ranges = ranges[:min_len]

        # Convertimos grados config a radianes
        lim_min_rad = np.radians(cfg['ang_min'])
        lim_max_rad = np.radians(cfg['ang_max'])

        # AQUI ESTA LA CORRECCIÓN DEL ÁNGULO:
        # Filtramos ANTES de calcular XYZ
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

    def crear_referencia_altura(self, z, color):
        # Dibuja un cuadrado flotante a la altura Z indicada
        points = [
            [-5*self.SCALE, -2*self.SCALE, z], [ 5*self.SCALE, -2*self.SCALE, z],
            [-5*self.SCALE,  2*self.SCALE, z], [ 5*self.SCALE,  2*self.SCALE, z]
        ]
        lines = [[0,1], [1,3], [3,2], [2,0]]
        ls = o3d.geometry.LineSet()
        ls.points = o3d.utility.Vector3dVector(points)
        ls.lines = o3d.utility.Vector2iVector(lines)
        ls.paint_uniform_color(color)
        return ls

def main():
    rclpy.init()
    gui = SistemaLogicaPLC()
    t = threading.Thread(target=rclpy.spin, args=(gui,), daemon=True)
    t.start()
    try:
        while gui.running:
            if gui.new_data:
                gui.pcd_long.points = o3d.utility.Vector3dVector(gui.puntos_long)
                gui.pcd_long.paint_uniform_color([1, 0.5, 0]) 
                gui.vis.update_geometry(gui.pcd_long)
                gui.vis.update_geometry(gui.box_target) 
                gui.new_data = False
            
            gui.vis.poll_events(); gui.vis.update_renderer(); time.sleep(0.01)
    except KeyboardInterrupt: pass
    finally: gui.vis.destroy_window(); rclpy.shutdown()

if __name__ == '__main__': main()
