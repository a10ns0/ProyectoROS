import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Float64
import open3d as o3d
import numpy as np
import threading
import time
import json
import collections

# === LIBRERÍA PROFESIONAL DE GUI & PLOTTING ===
import dearpygui.dearpygui as dpg

class SistemaControlPortuario(Node):
    def __init__(self):
        super().__init__('sistema_control_portuario_v3')
        
        # ==========================================
        # 1. PARAMETROS DE ESCALA Y FÍSICA
        # ==========================================
        DISTANCIA_REAL_ORIGINAL = 20.4
        DISTANCIA_MAQUETA = 1.5 
        self.SCALE = DISTANCIA_MAQUETA / DISTANCIA_REAL_ORIGINAL

        # Dimensiones (Escaladas)
        self.AREA_LARGO = 20.40 * self.SCALE 
        self.AREA_ANCHO = 22.38 * self.SCALE
        self.CENTRO_OPERATIVO = 10.2 * self.SCALE
        
        # Filtros de Altura
        self.H_FILTRO_CONTENEDOR   = 3.0 * self.SCALE  
        self.H_FILTRO_CHASIS_HIGH  = 1.75 * self.SCALE 
        self.H_FILTRO_CHASIS_LOW   = 1.0 * self.SCALE  

        # ==========================================
        # 2. CONFIGURACIÓN SENSORES
        # ==========================================
        self.CFG_ESTRUC = {
            'pos': [-5.0 * self.SCALE, 10.2 * self.SCALE, 12.0 * self.SCALE],
            'dist_min': 0.1, 'dist_max': 30.0, 
            'ang_min': -45, 'ang_max': 180, 
            'pitch': np.radians(0), 'yaw': np.radians(-90), 'roll': np.radians(90)
        }
        
        self.CFG_LONG = {
            'pos': [22.38 * self.SCALE, 10.2 * self.SCALE, 4.0 * self.SCALE], 
            'dist_min': 0.1, 'dist_max': 50.0, 
            'ang_min': -90, 'ang_max': 90, 
            'pitch': np.radians(-90), 'yaw': np.radians(180), 'roll': 0 
        }

        # ==========================================
        # 3. ESTADO DEL SISTEMA & BUFFERS DE DATOS
        # ==========================================
        self.spreader_size = "40"       
        self.modo_operacion = "DESCARGA_DEL_BARCO"
        self.mensaje_estado = "ESPERANDO..."
        self.tipo_detectado = "NADA"
        
        self.distancia_medida = 0.0
        self.distancia_target = 0.0
        self.error_distancia  = 0.0

        # Buffers para Gráficas (Rolling Plot)
        self.max_samples = 500
        self.data_x = collections.deque(np.zeros(self.max_samples), maxlen=self.max_samples)
        self.data_y_dist = collections.deque(np.zeros(self.max_samples), maxlen=self.max_samples)
        self.data_y_target = collections.deque(np.zeros(self.max_samples), maxlen=self.max_samples)
        self.sample_count = 0

        # ROS
        self.create_subscription(String, 'grua/estado_completo', self.callback_api, 10)
        self.create_subscription(LaserScan, '/scan_distancia', self.callback_longitudinal, 10)
        self.create_subscription(LaserScan, '/scan_estructura', self.callback_estructura, 10)
        
        # Publishers Debug
        self.pub_distancia_graph = self.create_publisher(Float64, '/tps/distancia_real', 10)
        self.pub_error_graph     = self.create_publisher(Float64, '/tps/error_posicion', 10)
        
        # Visualización 3D
        self.puntos_long = np.zeros((0, 3))
        self.puntos_estruc = np.zeros((0, 3))
        self.new_data = False
        self.actualizar_caja = False
        
        # Inicializar GUI Target
        self.zona_target_min = np.array([0.0, 0.0, 0.0])
        self.zona_target_max = np.array([1.0, 1.0, 1.0])
        self.actualizar_target_frenado()

        # Iniciar Sistemas Visuales
        self.init_dearpygui() # <--- NUEVO DASHBOARD
        self.iniciar_gui_3d()
        self.running = True

    # ==========================================
    # NUEVA GUI PROFESIONAL (Dear PyGui)
    # ==========================================
    def init_dearpygui(self):
        dpg.create_context()
        
        # Tema Oscuro Profesional
        with dpg.theme() as global_theme:
            with dpg.theme_component(dpg.mvAll):
                dpg.add_theme_style(dpg.mvStyleVar_WindowRounding, 5)
                dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 3)
        dpg.bind_theme(global_theme)

        # Ventana de Control
        with dpg.window(label="TPS CONTROL DASHBOARD", width=600, height=800, pos=(0,0), tag="win_main"):
            
            # --- SECCIÓN SUPERIOR: ESTADO ---
            with dpg.group(horizontal=True):
                dpg.add_text("ESTADO SISTEMA:", color=(200, 200, 200))
                self.lbl_status = dpg.add_text("INICIANDO", color=(255, 255, 0))
            
            dpg.add_separator()
            
            with dpg.group(horizontal=True):
                # Columna Izquierda: Métricas
                with dpg.group(width=250):
                    dpg.add_text("Métricas en Tiempo Real")
                    with dpg.table(header_row=False):
                        dpg.add_table_column()
                        dpg.add_table_column()
                        
                        with dpg.table_row():
                            dpg.add_text("Distancia:")
                            self.lbl_dist = dpg.add_text("0.00 m", color=(0, 255, 255))
                        with dpg.table_row():
                            dpg.add_text("Objetivo:")
                            self.lbl_target = dpg.add_text("0.00 m")
                        with dpg.table_row():
                            dpg.add_text("Error:")
                            self.lbl_error = dpg.add_text("0.00 m")
                        with dpg.table_row():
                            dpg.add_text("Objeto:")
                            self.lbl_obj = dpg.add_text("---")

                # Columna Derecha: Controles Manuales
                with dpg.group():
                    dpg.add_text("Simulación PLC")
                    dpg.add_button(label="Alternar Spreader (20'/40')", callback=self.cb_toggle_spreader, width=-1)
                    dpg.add_button(label="Alternar Twistlocks (Carga/Descarga)", callback=self.cb_toggle_twistlock, width=-1)
                    self.lbl_mode = dpg.add_text(f"MODO: {self.spreader_size}' | {self.modo_operacion}", size=15)

            dpg.add_spacer(height=10)
            dpg.add_separator()

            # --- SECCIÓN GRÁFICA (PLOTTER) ---
            dpg.add_text("Telemetría de Aproximación")
            
            with dpg.plot(label="Distancia Laser vs Target", height=300, width=-1, anti_aliased=True):
                dpg.add_plot_legend()
                
                # Ejes
                self.xaxis = dpg.add_plot_axis(dpg.mvXAxis, label="Muestras", lock_min=True)
                self.yaxis = dpg.add_plot_axis(dpg.mvYAxis, label="Distancia (m)", lock_min=False)
                
                # Series de Datos
                # Línea Real (Cyan)
                self.series_dist = dpg.add_line_series([], [], label="Distancia Real", parent=self.yaxis)
                # Línea Target (Verde)
                self.series_target = dpg.add_line_series([], [], label="Target Point", parent=self.yaxis)
                
                # Estilo de líneas
                dpg.bind_item_theme(self.series_dist, self.crear_tema_linea((0, 255, 255)))
                dpg.bind_item_theme(self.series_target, self.crear_tema_linea((0, 255, 0)))

        dpg.create_viewport(title='HMI TPS System', width=600, height=600)
        dpg.setup_dearpygui()
        dpg.show_viewport()
        # NOTA: No llamamos a dpg.start_dearpygui() porque usaremos el modo manual en el loop principal

    def crear_tema_linea(self, color):
        with dpg.theme() as t:
            with dpg.theme_component(dpg.mvLineSeries):
                dpg.add_theme_color(dpg.mvPlotCol_Line, color, category=dpg.mvThemeCat_Plots)
                dpg.add_theme_style(dpg.mvStyleVar_PlotLinesWeight, 2, category=dpg.mvThemeCat_Plots)
        return t

    def actualizar_gui_profesional(self):
        # 1. Actualizar Textos
        dist_real_m = self.distancia_medida / self.SCALE
        target_real_m = self.distancia_target / self.SCALE
        err_real_m = self.error_distancia / self.SCALE
        
        dpg.set_value(self.lbl_dist, f"{dist_real_m:.3f} m")
        dpg.set_value(self.lbl_target, f"{target_real_m:.3f} m")
        dpg.set_value(self.lbl_error, f"{err_real_m:.3f} m")
        dpg.set_value(self.lbl_obj, self.tipo_detectado)
        
        dpg.set_value(self.lbl_status, self.mensaje_estado)
        # Cambiar color status dinamicamente
        color_status = (200, 200, 200)
        if "STOP" in self.mensaje_estado: color_status = (0, 255, 0)
        elif "AVANCE" in self.mensaje_estado: color_status = (255, 255, 0)
        elif "ATRAS" in self.mensaje_estado: color_status = (255, 0, 0)
        dpg.configure_item(self.lbl_status, color=color_status)

        dpg.set_value(self.lbl_mode, f"MODO: {self.spreader_size}' | {self.modo_operacion}")

        # 2. Actualizar Gráfica (Rolling Buffer)
        self.sample_count += 1
        self.data_x.append(self.sample_count)
        self.data_y_dist.append(dist_real_m)
        self.data_y_target.append(target_real_m)

        # Enviar arrays a DPG
        dpg.set_value(self.series_dist, [list(self.data_x), list(self.data_y_dist)])
        dpg.set_value(self.series_target, [list(self.data_x), list(self.data_y_target)])
        
        # Autoajuste del eje X para efecto "rolling"
        if self.sample_count > self.max_samples:
            dpg.set_axis_limits(self.xaxis, self.sample_count - self.max_samples, self.sample_count)
        else:
            dpg.set_axis_limits(self.xaxis, 0, self.max_samples)
            
        # Renderizar Frame DPG
        dpg.render_dearpygui_frame()

    # ==========================================
    # CALLBACKS DE BOTONES GUI
    # ==========================================
    def cb_toggle_spreader(self):
        self.spreader_size = "40" if self.spreader_size == "20" else "20"
        self.actualizar_target_frenado()
        
    def cb_toggle_twistlock(self):
        if self.modo_operacion == "CARGA_AL_BARCO":
            self.modo_operacion = "DESCARGA_DEL_BARCO"
        else:
            self.modo_operacion = "CARGA_AL_BARCO"

    # ==========================================
    # LÓGICA ROS & SENSORES (Igual a v2)
    # ==========================================
    def callback_api(self, msg):
        try:
            data = json.loads(msg.data)
            self.spreader_size = str(data.get("spreaderSize", "40"))
            twistlock = data.get("spreaderTwistlock", "UNLOCKED")
            self.modo_operacion = "CARGA_AL_BARCO" if twistlock == "LOCKED" else "DESCARGA_DEL_BARCO"
            self.actualizar_target_frenado()
        except ValueError: pass

    def actualizar_target_frenado(self):
        LARGO_CHASIS = 13.755 * self.SCALE
        LARGO_CABINA = 1.60 * self.SCALE
        DELTA_40 = 2.54 * self.SCALE
        
        distancia_sensor_a_centro = self.AREA_ANCHO - self.CENTRO_OPERATIVO
        mitad_largo_camion = (LARGO_CABINA + LARGO_CHASIS) / 2 
        target_base = distancia_sensor_a_centro - mitad_largo_camion 
        
        if self.spreader_size == "40":
            self.distancia_target = target_base - DELTA_40
        else:
            self.distancia_target = target_base
            
        center_x = self.AREA_ANCHO - self.distancia_target - mitad_largo_camion
        largo_zona = (12.2 * self.SCALE) if self.spreader_size == "40" else (6.1 * self.SCALE)
        
        self.zona_target_min = np.array([center_x - largo_zona/2, self.CENTRO_OPERATIVO - (1.5*self.SCALE), 0.0])
        self.zona_target_max = np.array([center_x + largo_zona/2, self.CENTRO_OPERATIVO + (1.5*self.SCALE), 4.0*self.SCALE])
        self.actualizar_caja = True

    def callback_longitudinal(self, msg):
        points = self.laser_to_xyz(msg, self.CFG_LONG)
        self.puntos_long = points
        
        ancho_calle = 2.0 * self.SCALE
        mask_calle = (points[:, 1] > (self.CENTRO_OPERATIVO - ancho_calle)) & \
                     (points[:, 1] < (self.CENTRO_OPERATIVO + ancho_calle))
        
        mask_altura = None
        if self.modo_operacion == 'CARGA_AL_BARCO':
            mask_altura = (points[:,2] > self.H_FILTRO_CONTENEDOR)
        else: 
            mask_altura = (points[:,2] > self.H_FILTRO_CHASIS_LOW) & \
                          (points[:,2] < self.H_FILTRO_CHASIS_HIGH)
               
        puntos_validos = points[mask_calle & mask_altura]
        
        if len(puntos_validos) > 5:
            lectura_sensor = np.min(np.linalg.norm(puntos_validos - np.array(self.CFG_LONG['pos']), axis=1))
            self.distancia_medida = lectura_sensor
            self.error_distancia = self.distancia_medida - self.distancia_target

            # Publishers para otras herramientas (opcional ahora que tenemos GUI propia)
            msg_dist = Float64(); msg_dist.data = float(self.distancia_medida)
            self.pub_distancia_graph.publish(msg_dist)
            msg_err = Float64(); msg_err.data = float(self.error_distancia)
            self.pub_error_graph.publish(msg_err)

            tolerancia = 0.20 * self.SCALE
            if abs(self.error_distancia) <= tolerancia:
                self.mensaje_estado = "STOP - OK"
            elif self.error_distancia > 0:
                self.mensaje_estado = f"AVANCE {abs(self.error_distancia/self.SCALE):.2f}m"
            else:
                self.mensaje_estado = f"ATRAS {abs(self.error_distancia/self.SCALE):.2f}m"
        else:
            self.distancia_medida = 0.0
            self.mensaje_estado = "BUSCANDO..."
            
        self.new_data = True

    def callback_estructura(self, msg):
        points = self.laser_to_xyz(msg, self.CFG_ESTRUC)
        self.puntos_estruc = points
        
        z_vals = points[:, 2]
        if len(z_vals) > 5:
            avg_h = np.mean(z_vals[z_vals > (0.5 * self.SCALE)]) 
            limit_chasis = 1.8 * self.SCALE
            limit_cabina = 2.8 * self.SCALE

            if avg_h <= limit_chasis: self.tipo_detectado = "CHASIS"
            elif avg_h > limit_cabina: self.tipo_detectado = "CONTENEDOR"
            else: self.tipo_detectado = "CABINA/OTRO"
        
        self.new_data = True

    # ==========================================
    # VISUALIZADOR 3D (OPEN3D)
    # ==========================================
    def iniciar_gui_3d(self):
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name="DIGITAL TWIN 3D", width=800, height=600, left=620, top=0) # Posicionado a la derecha
        
        axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=2.0 * self.SCALE)
        self.vis.add_geometry(axis)
        
        self.box_target = o3d.geometry.AxisAlignedBoundingBox(min_bound=self.zona_target_min, max_bound=self.zona_target_max)
        self.box_target.color = [0, 1, 0]
        
        self.pcd_long = o3d.geometry.PointCloud()
        self.pcd_estruc = o3d.geometry.PointCloud()
        
        self.vis.add_geometry(self.box_target)
        self.vis.add_geometry(self.pcd_long)
        self.vis.add_geometry(self.pcd_estruc)
        
        ctr = self.vis.get_view_control()
        ctr.set_lookat([self.AREA_ANCHO/2, self.AREA_LARGO/2, 0])
        ctr.set_front([0.0, 0.0, 1.0])
        ctr.set_up([0.0, 1.0, 0.0])
        ctr.set_zoom(0.5)

    def laser_to_xyz(self, msg, cfg):
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        ranges = np.array(msg.ranges)
        min_len = min(len(angles), len(ranges))
        angles = angles[:min_len]; ranges = ranges[:min_len]
        
        valid = (ranges > cfg['dist_min']) & (ranges < cfg['dist_max'])
        r = ranges[valid]; a = angles[valid]
        
        if len(r) == 0: return np.zeros((0, 3))
        
        x = r * np.cos(a); y = r * np.sin(a); z = np.zeros_like(x)
        
        if cfg['yaw'] != 0: 
            c, s = np.cos(cfg['yaw']), np.sin(cfg['yaw'])
            x_n, y_n = x*c - y*s, x*s + y*c
            x, y = x_n, y_n
        if cfg['pitch'] != 0: 
            c, s = np.cos(cfg['pitch']), np.sin(cfg['pitch'])
            x_n, z_n = x*c - z*s, x*s + z*c
            x, z = x_n, z_n
        if cfg['roll'] != 0:
            c, s = np.cos(cfg['roll']), np.sin(cfg['roll'])
            y_n, z_n = y*c - z*s, y*s + z*c
            y, z = y_n, z_n

        points = np.vstack((x + cfg['pos'][0], y + cfg['pos'][1], z + cfg['pos'][2])).T
        return points

def main(args=None):
    rclpy.init(args=args)
    gui = SistemaControlPortuario()
    t = threading.Thread(target=rclpy.spin, args=(gui,), daemon=True)
    t.start()
    
    try:
        # BUCLE PRINCIPAL HÍBRIDO (OPEN3D + DEARPYGUI)
        while gui.running and dpg.is_dearpygui_running():
            
            # 1. Actualizar Open3D (3D)
            if gui.new_data:
                gui.pcd_long.points = o3d.utility.Vector3dVector(gui.puntos_long)
                gui.pcd_long.paint_uniform_color([1, 0.5, 0]) 
                gui.pcd_estruc.points = o3d.utility.Vector3dVector(gui.puntos_estruc)
                gui.pcd_estruc.paint_uniform_color([0, 1, 1])
                gui.vis.update_geometry(gui.pcd_long)
                gui.vis.update_geometry(gui.pcd_estruc)
                gui.new_data = False
            
            if gui.actualizar_caja:
                gui.box_target.min_bound = gui.zona_target_min
                gui.box_target.max_bound = gui.zona_target_max
                gui.vis.update_geometry(gui.box_target)
                gui.actualizar_caja = False
            
            gui.vis.poll_events()
            gui.vis.update_renderer()

            # 2. Actualizar Dashboard Profesional (2D)
            gui.actualizar_gui_profesional()
            
            # Pequeño sleep para no fundir la CPU (opcional)
            # time.sleep(0.005) 

    except KeyboardInterrupt: pass
    finally:
        dpg.destroy_context()
        gui.vis.destroy_window()
        rclpy.shutdown()

if __name__ == '__main__':
    main()