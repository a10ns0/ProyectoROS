import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer
from tf2_ros.transform_listener import TransformListener
import open3d as o3d
import numpy as np
import threading
import time

class CalibradorManual(Node):
    def __init__(self):
        super().__init__('calibrador_manual_sts')
        
        # =========================================================
        # VALORES INICIALES (MODIFICALOS EN VIVO CON TECLADO)
        # =========================================================
        # Empezamos con los que tienes ahora (Guillotina) para que veas el cambio
        self.cur_yaw   = -90.0
        self.cur_pitch = 0.0
        self.cur_roll  = 90.0
        
        self.CFG_LONG = {
            'pos': [10.2, 0.0, 12.5], 
            'dist_min': 0.1, 'dist_max': 40,
            # Abrimos el angulo al maximo para ver todo
            'ang_min': -95.0, 'ang_max': 95.0, 
        }

        # Buffers
        self.puntos_long = np.zeros((0, 3))
        self.new_data = False

        # ROS Setup
        self.create_subscription(LaserScan, '/scan_distancia', self.cb_longitudinal, 10)
        self.pub_cmd = self.create_publisher(String, '/truck_cmd', 10)
        
        print("="*60)
        print("   MODO CALIBRACIÓN MANUAL - SENSOR 2")
        print("="*60)
        print("   [U] / [J] :  ROTAR YAW (Giro Izq/Der)")
        print("   [I] / [K] :  ROTAR PITCH (Inclinación Arriba/Abajo)")
        print("   [O] / [L] :  ROTAR ROLL (Giro sobre su eje)")
        print("   [W/S]     :  Mover Camión")
        print("="*60)

        self.init_gui()
        self.running = True

    def init_gui(self):
        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.vis.create_window("CALIBRADOR DE SENSOR", 1280, 720)
        self.vis.get_render_option().point_size = 4.0

        # --- CONTROLES DE CALIBRACIÓN ---
        # YAW
        self.vis.register_key_callback(ord("U"), lambda v: self.ajustar_angulo('yaw', 5))
        self.vis.register_key_callback(ord("J"), lambda v: self.ajustar_angulo('yaw', -5))
        # PITCH
        self.vis.register_key_callback(ord("I"), lambda v: self.ajustar_angulo('pitch', 5))
        self.vis.register_key_callback(ord("K"), lambda v: self.ajustar_angulo('pitch', -5))
        # ROLL
        self.vis.register_key_callback(ord("O"), lambda v: self.ajustar_angulo('roll', 5))
        self.vis.register_key_callback(ord("L"), lambda v: self.ajustar_angulo('roll', -5))
        
        # Movimiento Camión
        self.vis.register_key_callback(ord("W"), lambda v: self.pub_cmd.publish(String(data="FORWARD")))
        self.vis.register_key_callback(ord("S"), lambda v: self.pub_cmd.publish(String(data="BACKWARD")))
        self.vis.register_key_callback(ord(" "), lambda v: self.pub_cmd.publish(String(data="STOP")))

        # Geometrías
        self.pcd_long = o3d.geometry.PointCloud()
        self.vis.add_geometry(self.pcd_long)
        self.vis.add_geometry(self.crear_suelo())
        
        # Referencia Sensor
        s1 = o3d.geometry.TriangleMesh.create_sphere(radius=0.5)
        s1.translate(self.CFG_LONG['pos'])
        s1.paint_uniform_color([1, 0, 0])
        self.vis.add_geometry(s1)

        # Camara
        ctr = self.vis.get_view_control()
        ctr.set_lookat([0,0,0]); ctr.set_front([0.0, -0.1, 1.0]); ctr.set_zoom(0.4)

    def ajustar_angulo(self, eje, delta):
        if eje == 'yaw': self.cur_yaw += delta
        elif eje == 'pitch': self.cur_pitch += delta
        elif eje == 'roll': self.cur_roll += delta
        
        print(f">>> AJUSTE ACTUAL: Yaw={self.cur_yaw:.1f} | Pitch={self.cur_pitch:.1f} | Roll={self.cur_roll:.1f}")

    def cb_longitudinal(self, msg):
	
        # Usamos los valores dinámicos (self.cur_yaw, etc)
        points = self.laser_to_xyz_dinamico(msg, self.CFG_LONG)
        self.puntos_long = points
        self.new_data = True

    def laser_to_xyz_dinamico(self, msg, cfg):
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        ranges = np.array(msg.ranges)
        min_len = min(len(angles), len(ranges))
        angles = angles[:min_len]; ranges = ranges[:min_len]

        # Filtro basico
        valid = (ranges > cfg['dist_min']) & (ranges < cfg['dist_max'])
        r = ranges[valid]; a = angles[valid]
        if len(r) == 0: return np.zeros((0, 3))

        x = r * np.cos(a); y = r * np.sin(a); z = np.zeros_like(x)
        points = np.vstack((x, y, z)).T

        # ROTACIÓN DINÁMICA CON TUS TECLAS
        R = o3d.geometry.get_rotation_matrix_from_xyz((
            np.radians(self.cur_roll), 
            np.radians(self.cur_pitch), 
            np.radians(self.cur_yaw)
        ))
        points = points @ R.T
        points += np.array(cfg['pos'])
        return points

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

def main():
    rclpy.init()
    gui = CalibradorManual()
    t = threading.Thread(target=rclpy.spin, args=(gui,), daemon=True)
    t.start()
    try:
        while gui.running:
            if gui.new_data:
                gui.pcd_long.points = o3d.utility.Vector3dVector(gui.puntos_long)
                gui.pcd_long.paint_uniform_color([1, 0.5, 0]) 
                gui.vis.update_geometry(gui.pcd_long)
                gui.new_data = False
            
            gui.vis.poll_events(); gui.vis.update_renderer(); time.sleep(0.01)
    except KeyboardInterrupt: pass
    finally: gui.vis.destroy_window(); rclpy.shutdown()

if __name__ == '__main__': main()
