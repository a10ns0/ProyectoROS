import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import math

class AsistenteEstacionamiento(Node):
    def __init__(self):
        super().__init__('asistente_estacionamiento')
        
        # Suscripción al tópico del láser
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
        # -----------------------------------------------------------
        # ZONA DE CONFIGURACIÓN (PRUEBAS A 15 METROS)
        # -----------------------------------------------------------
        self.altura_sensor = 15.0        # Altura actual de pruebas
        self.angulo_inclinacion_deg = 31.0 # Ángulo calculado para ver a 25m
        
        self.meta_centro = 12.5          # El objetivo: centro del camión a 12.5m
        self.tolerancia_centro = 0.5     # Margen de error aceptable (50cm)
        self.altura_minima_chasis = 2.5  # Altura mínima para ser camión
        
        # -----------------------------------------------------------
        
        # Pre-calculo de radianes
        self.angulo_inclinacion_rad = math.radians(self.angulo_inclinacion_deg)
        self.get_logger().info(f'Sistema Iniciado. Altura: {self.altura_sensor}m. Objetivo: {self.meta_centro}m')

    def scan_callback(self, msg):
        # 1. Preparar datos crudos del sensor
        rangos = np.array(msg.ranges)
        indices = np.arange(len(rangos))
        angulos_laser = msg.angle_min + (indices * msg.angle_increment)
        
        # Ángulo total respecto a la horizontal
        angulos_totales = self.angulo_inclinacion_rad + angulos_laser
        
        # 2. Calcular Coordenadas Reales (X y Z)
        # Z = Altura del objeto desde el suelo
        # X = Distancia horizontal hacia adelante desde el sensor
        Z_detectado = self.altura_sensor - (rangos * np.sin(angulos_totales))
        X_detectado = rangos * np.cos(angulos_totales)
        
        # 3. FILTRO DE DATOS (AQUÍ ESTÁ LA PROTECCIÓN DE LA VIGA)
        mask_chasis = (
            (np.isfinite(rangos)) &             # Dato numérico válido
            (rangos > 2.0) &                    # PROTECCIÓN 1: Ignora cosas a menos de 2m del sensor (la viga pegada)
            (Z_detectado > self.altura_minima_chasis) &  # Que sea más alto que un auto (> 2.5m)
            (Z_detectado < 6.0) &               # PROTECCIÓN 2: Ignora cosas altísimas (parte de la grúa > 6m)
            (X_detectado > 0) &                 # Que esté delante del sensor
            (X_detectado < 25.0)                # Que esté dentro de la zona de trabajo (0-25m)
        )
        
        # Extraemos solo los valores de X (distancia) que pasaron el filtro
        puntos_chasis_x = X_detectado[mask_chasis]
        
        # Si detectamos muy pocos puntos, asumimos que no hay camión
        if len(puntos_chasis_x) < 10:
            # self.get_logger().info("Zona despejada o esperando camión...", throttle_duration_sec=2)
            return

        # 4. Calcular la posición del camión
        inicio_chasis = np.min(puntos_chasis_x) # Punto más cercano
        fin_chasis = np.max(puntos_chasis_x)    # Punto más lejano
        
        # El centro es el promedio entre el inicio y el fin
        centro_actual = (inicio_chasis + fin_chasis) / 2.0
        largo_detectado = fin_chasis - inicio_chasis
        
        # 5. Lógica de Semaforo / Estacionamiento
        diferencia = centro_actual - self.meta_centro
        
        # Mensaje base
        estado = ""
        
        if abs(diferencia) <= self.tolerancia_centro:
            estado = "✅ STOP - ESTACIONADO"
            # AQUÍ ENVIARÍAS LA SEÑAL DE PARADA AL PLC
            
        elif diferencia > 0:
            # El centro está en 14m, 15m, etc. (Se pasó o está entrando mal)
            estado = f"⚠️ RETROCEDER (Sobran {diferencia:.2f}m)"
            
        else: # diferencia < 0
            # El centro está en 8m, 10m... (Le falta avanzar)
            estado = f"⬇️ AVANZAR (Faltan {abs(diferencia):.2f}m)"

        # Imprimir en consola
        self.get_logger().info(f"Camión detectado ({largo_detectado:.1f}m largo) | Pos: {centro_actual:.1f}m | {estado}")

def main(args=None):
    rclpy.init(args=args)
    nodo = AsistenteEstacionamiento()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()