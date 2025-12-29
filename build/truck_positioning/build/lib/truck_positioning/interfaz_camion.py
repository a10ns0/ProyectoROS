import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import matplotlib.pyplot as plt
import numpy as np

class VisualizadorDoble(Node):
    def __init__(self):
        super().__init__('visualizador_doble_sensor')
        
        # ====================================================================
        #                 👇 ZONA DE CONFIGURACIÓN AVANZADA 👇
        # ====================================================================
        
        # --- CONFIGURACIÓN SENSOR 1 (Izquierda / Estructura) ---
        self.topic_sensor_1 = '/scan_estructura'
        self.s1_distancia_max = 1.0  # Metros máximos
        self.s1_angulo_izq    = 20.0  # Grados hacia la IZQUIERDA (Positivo)
        self.s1_angulo_der    = 20.0  # Grados hacia la DERECHA (Negativo)

        # --- CONFIGURACIÓN SENSOR 2 (Derecha / Distancia) ---
        self.topic_sensor_2 = '/scan_distancia'
        self.s2_distancia_max = 7.0   # Metros máximos (Puede ser distinto al 1)
        self.s2_angulo_izq    = 10.0  # Ejemplo: Ve poco a la izquierda...
        self.s2_angulo_der    = 70.0  # ...pero ve mucho a la derecha.

        # NOTA: En ROS, 0 grados es el frente. Izquierda es (+), Derecha es (-).
        # Aquí solo pon los números positivos, el código hace la conversión.
        
        # ====================================================================
        #                 👆 FIN DE ZONA DE CONFIGURACIÓN 👆
        # ====================================================================
        
        # Suscripciones
        self.sub1 = self.create_subscription(
            LaserScan, self.topic_sensor_1, self.callback_sensor_1, 10)
            
        self.sub2 = self.create_subscription(
            LaserScan, self.topic_sensor_2, self.callback_sensor_2, 10)
        
        self.x1, self.y1 = [], []
        self.x2, self.y2 = [], []
        self.nuevo_dato_1 = False
        self.nuevo_dato_2 = False

    def procesar_scan(self, msg, dist_max, ang_izq_grad, ang_der_grad):
        """
        Procesa el scan con límites personalizados pasados como argumentos.
        """
        xs = []
        ys = []
        
        # Convertimos grados a radianes para comparar con ROS
        # Límite Izquierdo es positivo (+), Límite Derecho es negativo (-)
        limite_rad_izq = math.radians(ang_izq_grad)
        limite_rad_der = -math.radians(ang_der_grad) # Lo volvemos negativo

        for i, rango in enumerate(msg.ranges):
            # 1. Filtro de errores
            if rango == float('inf') or rango == 0.0:
                continue
            
            # 2. Filtro de DISTANCIA (Independiente)
            if rango > dist_max:
                continue

            angulo = msg.angle_min + (i * msg.angle_increment)
            
            # 3. Filtro de ÁNGULO (Asimétrico)
            # El ángulo debe ser MENOR que el límite izq Y MAYOR que el límite der (que es negativo)
            if angulo > limite_rad_izq or angulo < limite_rad_der:
                continue

            # Conversión a Cartesianas
            x = rango * math.cos(angulo)
            y = rango * math.sin(angulo)
            xs.append(x)
            ys.append(y)
            
        return xs, ys

    def callback_sensor_1(self, msg):
        # Al llamar a procesar, le pasamos la configuración del SENSOR 1
        self.x1, self.y1 = self.procesar_scan(
            msg, 
            self.s1_distancia_max, 
            self.s1_angulo_izq, 
            self.s1_angulo_der
        )
        self.nuevo_dato_1 = True

    def callback_sensor_2(self, msg):
        # Al llamar a procesar, le pasamos la configuración del SENSOR 2
        self.x2, self.y2 = self.procesar_scan(
            msg, 
            self.s2_distancia_max, # OJO: Aquí uso la variable del sensor 2
            self.s2_angulo_izq, 
            self.s2_angulo_der
        )
        self.nuevo_dato_2 = True

def main(args=None):
    rclpy.init(args=args)
    nodo = VisualizadorDoble()

    plt.ion()
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))
    
    # --- GRÁFICO 1 (Configurado con datos del Sensor 1) ---
    limite_vis_1 = nodo.s1_distancia_max + 1.0
    scat1 = ax1.scatter([], [], s=10, c='magenta', label='Sensor 1')
    ax1.set_xlim(-limite_vis_1/2, limite_vis_1/2) 
    ax1.set_ylim(-1, limite_vis_1)                
    ax1.set_title(f'SENSOR 1\nIzq:{nodo.s1_angulo_izq}° | Der:{nodo.s1_angulo_der}°')
    ax1.set_xlabel('Metros')
    ax1.set_ylabel('Metros')
    ax1.grid(True)
    ax1.plot(0, 0, 'ko', markersize=10) # Punto del sensor

    # --- GRÁFICO 2 (Configurado con datos del Sensor 2) ---
    limite_vis_2 = nodo.s1_distancia_max + 1.0 # Usamos la dist del sensor 2
    scat2 = ax2.scatter([], [], s=10, c='gold', label='Sensor 2') 
    ax2.set_xlim(-limite_vis_2/2, limite_vis_2/2)
    ax2.set_ylim(-1, limite_vis_2)
    ax2.set_title(f'SENSOR 2\nIzq:{nodo.s2_angulo_izq}° | Der:{nodo.s2_angulo_der}°')
    ax2.set_xlabel('Metros')
    ax2.set_ylabel('Metros')
    ax2.grid(True)
    ax2.plot(0, 0, 'ko', markersize=10)

    try:
        while rclpy.ok():
            rclpy.spin_once(nodo, timeout_sec=0.1)

            if nodo.nuevo_dato_1:
                puntos1 = np.c_[nodo.x1, nodo.y1]
                if len(puntos1) > 0: scat1.set_offsets(puntos1)
                else: scat1.set_offsets(np.empty((0, 2)))
                nodo.nuevo_dato_1 = False
            
            if nodo.nuevo_dato_2:
                puntos2 = np.c_[nodo.x2, nodo.y2]
                if len(puntos2) > 0: scat2.set_offsets(puntos2)
                else: scat2.set_offsets(np.empty((0, 2)))
                nodo.nuevo_dato_2 = False

            fig.canvas.draw_idle()
            plt.pause(0.01)

    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()
        plt.close()

if __name__ == '__main__':
    main()
