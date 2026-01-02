import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import open3d as o3d
import numpy as np
import threading
import time
import json
import cv2



class SistemaControlPortuario(Node):
    def __init__(self):
        super().__init__('sistema_control_portuario')
        
        # ==========================================
        # 1. PARAMETROS FÍSICOS (SEGÚN TU PLANO)
        # ==========================================
        # Dimensiones de la Grúa / Área
        self.AREA_LARGO = 20.40  # Eje Y (Ancho del carril)
        self.AREA_ANCHO = 22.38  # Eje X (Largo del recorrido)
        self.CENTRO_OPERATIVO = 10.2 # Punto medio (Donde cae el Spreader)
        
        # Dimensiones Camión (Filtros)
        self.ALTURA_CHASIS = 1.57      # Metros
        self.ALTURA_CABINA = 3.10      # > 3 metros
        self.LARGO_CHASIS  = 13.755
        self.LARGO_CABINA  = 1.60
        
        # Lógica de Posicionamiento (DELTA)
        self.DELTA_40_PIES = 2.54 # Metros de desplazamiento solicitado
        
        # ==========================================
        # 2. CONFIGURACIÓN DE SENSORES
        # ==========================================
        # Sensor 1 (Lateral/Perfil): Ubicado al costado, escanea el perfil transversal
        self.CFG_ESTRUC = {
            'pos': [-0, 10.2, 12.5], # Ubicado lateralmente, centrado en Y
            'dist_min': 0.1, 'dist_max': 30.0, 
            'ang_min': -45, 'ang_max': 180, 
            'pitch': np.radians(-90), 'yaw': np.radians(90), 'roll': 0
        }
        
        # Sensor 2 (Longitudinal): Ubicado al FINAL del carril (X=22.38), mirando hacia la entrada (X=0)
        self.CFG_LONG = {
            'pos': [22.38, 10.2, 12.5], # En el fondo, altura media
            'dist_min': 0.1, 'dist_max': 50.0, 
            'ang_min': -90, 'ang_max': 90, 
            'pitch': np.radians(0), 'yaw': np.radians(180), 'roll': 0 # Mirando hacia atrás
        }

        # ==========================================
        # 3. ESTADO DEL SISTEMA
        # ==========================================
        self.spreader_size = "20"       # Default seguro
        self.spreader_lock = "UNLOCKED" 
        self.modo_operacion = "ESPERA"
        
        self.distancia_medida = 0.0
        self.distancia_target = 0.0
        self.error_distancia  = 99.0
        
        self.mensaje_estado = "ESPERANDO..."
        self.color_estado = (100, 100, 100)
        self.tipo_detectado = "NADA" # NADA, CHASIS, CABINA, CONTAINER

        # Suscripciones
        self.create_subscription(String, 'grua/estado_completo', self.callback_api, 10)
        self.create_subscription(LaserScan, '/scan_distancia', self.callback_longitudinal, 10)
        self.create_subscription(LaserScan, '/scan_estructura', self.callback_estructura, 10)

        # Variables Visualización
        self.puntos_long = np.zeros((0, 3))
        self.puntos_estruc = np.zeros((0, 3))
        self.new_data = False
        self.actualizar_caja = False
        
        self.zona_target_min = np.array([8.0, 8.0, 0.0])
        self.zona_target_max = np.array([12.0, 12.0, 4.0])

        self.iniciar_gui_3d()
        self.running = True

    # ==========================================
    # LÓGICA PRINCIPAL (CEREBRO)
    # ==========================================
    def callback_api(self, msg):
        try:
            data = json.loads(msg.data)
            nuevo_size = str(data.get("spreaderSize", "20"))
            twistlock = data.get("spreaderTwistlock", "UNLOCKED")
            
            # Detectar cambios
            if nuevo_size != self.spreader_size:
                self.spreader_size = nuevo_size
                print(f"--> CAMBIO MODO: {self.spreader_size} PIES")
                self.actualizar_target_frenado()

            if twistlock == "LOCKED":
                self.modo_operacion = "RECEPCION (Pick)" # Viene a buscar
            else:
                self.modo_operacion = "ENTREGA (Drop)"   # Viene a dejar

        except ValueError: pass

    def actualizar_target_frenado(self):
        # CALCULO DE LA POSICIÓN DE PARADA IDEAL
        # El sensor está en X = 22.38m. El camión avanza desde X=0 hacia X=22.38.
        # Queremos que el CENTRO del camión quede en el CENTRO OPERATIVO (10.2m).
        
        # Distancia base desde el sensor hasta el centro operativo
        distancia_sensor_a_centro = self.AREA_ANCHO - self.CENTRO_OPERATIVO # 22.38 - 10.2 = 12.18m
        
        # Pero el sensor mide hasta el PARACHOQUES (Frente), no hasta el centro del camión.
        # Asumimos distancia del Parachoques al Centro del Camión:
        mitad_largo_camion = (self.LARGO_CABINA + self.LARGO_CHASIS) / 2 # Aprox 7.6m
        
        # Target Base (Para 20 pies - Estacionamiento centrado)
        # El sensor debería leer: (Distancia al centro) - (Mitad del camión)
        target_base = distancia_sensor_a_centro - mitad_largo_camion 
        
        if self.spreader_size == "40":
            # APLICACIÓN DEL DELTA DE 2.54 METROS
            # Si es 40 pies, desplazamos el punto de parada.
            # Asumimos desplazamiento hacia adelante (menor distancia al sensor) para alinear atrás.
            self.distancia_target = target_base - self.DELTA_40_PIES
        else:
            # Caso 20 pies: Ingresa completo al centro
            self.distancia_target = target_base
            
        # Actualizar caja visual (zona verde)
        center_x = self.AREA_ANCHO - self.distancia_target - mitad_largo_camion
        largo_zona = 12.2 if self.spreader_size == "40" else 6.1
        
        self.zona_target_min = np.array([center_x - largo_zona/2, self.CENTRO_OPERATIVO - 1.5, 0.0])
        self.zona_target_max = np.array([center_x + largo_zona/2, self.CENTRO_OPERATIVO + 1.5, 4.0])
        self.actualizar_caja = True

    def callback_longitudinal(self, msg):
        """ Sensor X=22.38: Mide distancia al frente del camión """
        points = self.laser_to_xyz(msg, self.CFG_LONG)
        self.puntos_long = points
        
        # Filtrar solo lo que está en el carril (Y entre 8m y 12m aprox) y altura razonable
        mask = (points[:, 1] > (self.CENTRO_OPERATIVO - 2.0)) & \
               (points[:, 1] < (self.CENTRO_OPERATIVO + 2.0)) & \
               (points[:, 2] > 0.5) # Ignorar suelo
               
        puntos_validos = points[mask]
        
        if len(puntos_validos) > 10:
            # El punto más cercano al sensor (menor X local, pero como el sensor mira atrás...)
            # En coordenadas globales, el camión viene de 0 a 22. 
            # El sensor está en 22 mirando a 0. La lectura es directa en metros.
            lectura_sensor = np.min(np.linalg.norm(puntos_validos - np.array(self.CFG_LONG['pos']), axis=1))
            
            self.distancia_medida = lectura_sensor
            self.error_distancia = self.distancia_medida - self.distancia_target
            
            # Semáforo
            tolerancia = 0.20 # 20 cm de precisión
            
            if abs(self.error_distancia) <= tolerancia:
                self.mensaje_estado = "STOP - OK"
                self.color_estado = (0, 255, 0) # Verde
            elif self.error_distancia > 0:
                self.mensaje_estado = f"AVANCE {abs(self.error_distancia):.2f}m"
                self.color_estado = (0, 255, 255) # Amarillo
            else:
                self.mensaje_estado = f"ATRAS {abs(self.error_distancia):.2f}m"
                self.color_estado = (0, 0, 255) # Rojo
        else:
            self.distancia_medida = 0.0
            self.mensaje_estado = "ESPERANDO CAMION"
            self.color_estado = (100, 100, 100)
            
        self.new_data = True

    def callback_estructura(self, msg):
        """ Sensor Lateral: Valida Alturas (Chasis vs Cabina vs Container) """
        points = self.laser_to_xyz(msg, self.CFG_ESTRUC)
        self.puntos_estruc = points
        
        # Analizar alturas promedio en la zona central
        z_vals = points[:, 2]
        
        if len(z_vals) > 0:
            max_h = np.max(z_vals)
            avg_h = np.mean(z_vals[z_vals > 0.5]) # Promedio sin suelo
            
            # Lógica de Clasificación por Altura (Tus datos)
            if 1.4 <= avg_h <= 1.8: # Chasis es 1.57m
                self.tipo_detectado = "CHASIS (VACIO)"
            elif avg_h > 2.8:       # Cabina > 3m o Container
                self.tipo_detectado = "CAMION/CARGA"
            else:
                self.tipo_detectado = "DESCONOCIDO"
        
        self.new_data = True

    # ==========================================
    # VISUALIZACIÓN HMI (2D + 3D)
    # ==========================================
    def renderizar_panel_2d(self):
        panel = np.zeros((450, 700, 3), dtype=np.uint8)
        
        # Encabezado
        cv2.putText(panel, f"MODO SPREADER: {self.spreader_size} PIES", (30, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(panel, f"OBJETIVO: {self.tipo_detectado}", (400, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
        
        # Datos Numéricos
        cv2.putText(panel, f"Target Sensor: {self.distancia_target:.2f} m", (30, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (150, 150, 150), 1)
        if self.spreader_size == "40":
            cv2.putText(panel, f"(Incluye Delta 2.54m)", (30, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            
        # Barra Gráfica de Distancia
        start_x, end_x = 50, 650
        cv2.line(panel, (start_x, 250), (end_x, 250), (100, 100, 100), 10) # Riel
        
        # Dibujar Meta
        target_px = int(start_x + (self.distancia_target / 30.0) * (end_x - start_x))
        cv2.line(panel, (target_px, 220), (target_px, 280), (0, 255, 0), 4)
        cv2.putText(panel, "META", (target_px - 20, 210), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        # Dibujar Camión Actual
        if self.distancia_medida > 0:
            camion_px = int(start_x + (self.distancia_medida / 30.0) * (end_x - start_x))
            cv2.circle(panel, (camion_px, 250), 15, self.color_estado, -1)
            cv2.putText(panel, f"{self.distancia_medida:.2f}m", (camion_px - 20, 290), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        # Semáforo Gigante
        cv2.rectangle(panel, (0, 350), (700, 450), self.color_estado, -1)
        cv2.putText(panel, self.mensaje_estado, (180, 410), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 0), 3)
        
        cv2.imshow("HMI OPERADOR PORTUARIO", panel)
        cv2.waitKey(1)

    def iniciar_gui_3d(self):
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name="DIGITAL TWIN 3D", width=800, height=600)
        
        # Ejes y Suelo
        axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=2.0)
        self.vis.add_geometry(axis)
        
        # Caja Target (Verde/Roja según estado)
        self.box_target = o3d.geometry.AxisAlignedBoundingBox(min_bound=self.zona_target_min, max_bound=self.zona_target_max)
        self.box_target.color = [0, 1, 0]
        
        # Nubes de Puntos
        self.pcd_long = o3d.geometry.PointCloud()
        self.pcd_estruc = o3d.geometry.PointCloud()
        
        self.vis.add_geometry(self.box_target)
        self.vis.add_geometry(self.pcd_long)
        self.vis.add_geometry(self.pcd_estruc)
        
        # Cámara Cenital (Vista planta como tu dibujo)
        ctr = self.vis.get_view_control()
        ctr.set_lookat([self.AREA_ANCHO/2, self.AREA_LARGO/2, 0])
        ctr.set_front([0.0, 0.0, 1.0]) # Desde arriba
        ctr.set_up([0.0, 1.0, 0.0])
        ctr.set_zoom(0.5)

    def laser_to_xyz(self, msg, cfg):
        # Conversión Polar -> Cartesiana + Transformación Homogénea (Rotación/Traslación)
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        ranges = np.array(msg.ranges)
        min_len = min(len(angles), len(ranges))
        angles = angles[:min_len]; ranges = ranges[:min_len]
        
        valid = (ranges > cfg['dist_min']) & (ranges < cfg['dist_max'])
        r = ranges[valid]; a = angles[valid]
        
        if len(r) == 0: return np.zeros((0, 3))
        
        # Local
        x = r * np.cos(a); y = r * np.sin(a); z = np.zeros_like(x)
        
        # Rotación Euler (Manual optimizada)
        if cfg['yaw'] != 0: # Rotar en Z
            c, s = np.cos(cfg['yaw']), np.sin(cfg['yaw'])
            x_n, y_n = x*c - y*s, x*s + y*c
            x, y = x_n, y_n
            
        if cfg['pitch'] != 0: # Rotar en Y (Para poner sensores verticales)
            c, s = np.cos(cfg['pitch']), np.sin(cfg['pitch'])
            x_n, z_n = x*c - z*s, x*s + z*c
            x, z = x_n, z_n
            
        if cfg['roll'] != 0: # Rotar en X
            c, s = np.cos(cfg['roll']), np.sin(cfg['roll'])
            y_n, z_n = y*c - z*s, y*s + z*c
            y, z = y_n, z_n

        # Traslación Global
        points = np.vstack((x + cfg['pos'][0], y + cfg['pos'][1], z + cfg['pos'][2])).T
        return points

def main(args=None):
    rclpy.init(args=args)
    gui = SistemaControlPortuario()
    t = threading.Thread(target=rclpy.spin, args=(gui,), daemon=True)
    t.start()
    
    try:
        while gui.running:
            if gui.new_data:
                # Colores: Naranja (Posición), Cyan (Perfil)
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
            gui.renderizar_panel_2d()
            time.sleep(0.01)
            
    except KeyboardInterrupt: pass
    finally:
        gui.vis.destroy_window()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()