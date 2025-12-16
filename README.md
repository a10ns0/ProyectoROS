# ProyectoROS
 Proyecto Sensor SICK Deteccion de distancia
 
Autores: 
**Cristobal
**Alonso Castillo Pinto
 
## Proyecto: Sistema de Asistencia al Posicionamiento de Camiones con SICK LMS251 y ROS2

##  Descripción
Este proyecto implementa un sistema de detección de distancia y evaluación de campos (bloques) utilizando el sensor **SICK LMS251** integrado en **ROS2**.  
El objetivo es asistir en el posicionamiento de camiones mediante lógica de zonas de seguridad (STOP / Aproximación) y retroalimentación visual en **RViz2**.

##  Tecnologías
- **Hardware:** Sensor LiDAR SICK LMS251
- **Software:** ROS2 (Humble/Foxy), SOPAS ET, RViz2
- **Driver:** [`sick_scan_xd`](https://github.com/SICKAG/sick_scan_xd)
- **Lenguaje:** Python (rclpy)

---

## Estructura del Proyecto

<img width="472" height="164" alt="image" src="https://github.com/user-attachments/assets/6a996eda-8da9-4625-ad73-715e62aa9a74" />

---

##  Fases de Puesta en Marcha

###  Fase 1: Configuración del Hardware
1. Conectar el sensor LMS251 a **24V DC** (Marrón = +, Azul = -).
2. Conectar el cable Ethernet al PC (Windows).
3. Configurar IP del PC en el mismo rango (ej. `192.168.0.10` / máscara `255.255.255.0`).
4. Abrir **SOPAS ET**:
   - Dibujar **Campos de Evaluación**:
     - Campo 1 (STOP): rectángulo pequeño (ej. 4–5 m).
     - Campo 2 (Aproximación): rectángulo más grande (ej. 5–15 m).
   - Activar telegrama **LFErec** en Interfaces → Ethernet.
   - Guardar configuración permanente.

---

### Fase 2: Preparación del Entorno ROS2
1. Crear workspace:
   ```bash
   mkdir -p ~/ProyectoROS/src && cd ~/ros2_ws/src

2. Instal driver:
   ```bash
   git clone https://github.com/SICKAG/sick_scan_xd.git

 3. Crear paquete personalizado:
    ```bash
    ros2 pkg create --build-type ament_python truck_positioning

 4. Compilar:
    ```bash
    cd ~/ProyectoROS
    colcon build
    source install/setup.bash

### Fase 3: Desarrollo de Nodos

1) Nodo Publisher (truck_block_logic.py)

* Suscripción: /scan (LaserScan), /sick_scan/lferec (bloques).

* Procesa nube de puntos y evalúa campos.

* Publica:

   /viz/blocks → MarkerArray (visualización de bloques).
   
   /viz/feedback_text → Marker (texto de retroalimentación).


2) Nodo Subscriber (truck_hmi.py)

* Suscripción: /viz/feedback_text.

* Muestra mensajes en consola (ej. “Retrocede 2.5 m”).



### Fase 4: Archivo de Lanzamiento

system.launch.py:

1) Inicia:

* Driver sick_scan_xd.

* Nodo de lógica truck_block_logic.

* Nodo HMI truck_hmi.

* RViz2 con configuración predefinida.


### Fase 5: Ejecución y Visualización

1. Compilar nuevamente:
   ```bash
   cd ~/ProyectoROS
   colcon build --packages-select truck_positioning
   source install/setup.bash

2. Lanzar sistema:
   ```bash
   ros2 launch truck_positioning system.launch.py

3. Configurar RVz2:

   Fixed Frame: cloud
   
   LaserScan: /scan
   
   MarkerArray: /viz/blocks
   
   Marker: /viz/feedback_text
   

4. Resultado:

* Cubos verdes → campos libres.

* Cubos rojos → campos ocupados.

* Texto flotante → instrucciones al operador (“Avanza X m”, “Retrocede Y m”).



