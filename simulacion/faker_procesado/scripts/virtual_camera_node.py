#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
import json
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PoseArray, PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

class VirtualCameraNode(Node):
    def __init__(self):
        super().__init__('virtual_camera_node')
        
        # --- CONFIGURACIÓN ---
        self.br = CvBridge()
        self.res_w, self.res_h = 640, 480
        self.focal_dist = 1.5
        self.sensor_w, self.sensor_h = 1.6, 1.2
        
        # Estado
        self.dron_pose = None
        self.cam_pose = None
        self.paneles_reales = [] 
        self.impactos_recientes = []

        # --- CLIENTE DE SERVICIO ---
        self.cli_realidad = self.create_client(Trigger, 'get_panel_real')

        # --- PUBLICADORES ---
        self.pub_imagen = self.create_publisher(Image, '/virtual_camera/image', 10)

        # --- SUSCRIPCIONES ---
        self.create_subscription(String, '/sim_status/panel_updates', self.actualizar_paneles_callback, 10)
        self.create_subscription(PoseStamped, '/data/drone', self.dron_callback, 10)
        self.create_subscription(PoseStamped, '/data/camera', self.camara_callback, 10)
        self.create_subscription(String, '/inspection/raw_data', self.raw_data_callback, 10)

        # --- TIMER (20 FPS) ---
        self.create_timer(0.05, self.render_loop)

        # Pedir mapa inicial
        self.pedir_mapa_inicial()
        self.get_logger().info("Cámara Virtual lista. Lógica de visión parcial activada.")

    def pedir_mapa_inicial(self):
        if not self.cli_realidad.service_is_ready():
            return
        req = Trigger.Request()
        self.cli_realidad.call_async(req).add_done_callback(self.al_recibir_mapa_srv)

    def al_recibir_mapa_srv(self, futuro):
        try:
            res = futuro.result()
            self.paneles_reales = json.loads(res.message)
            self.get_logger().info(f"Cámara Virtual: Mapa inicial cargado ({len(self.paneles_reales)} paneles).")
        except Exception as e:
            self.get_logger().error(f"Error cargando mapa inicial: {e}")

    def actualizar_paneles_callback(self, msg):
        self.pedir_mapa_inicial()

    def dron_callback(self, msg): self.dron_pose = msg
    def camara_callback(self, msg): self.cam_pose = msg
    def raw_data_callback(self, msg):
        try:
            self.impactos_recientes = json.loads(msg.data)
        except:
            self.impactos_recientes = []

    # ==========================================
    # RENDER ENGINE
    # ==========================================

    def render_loop(self):
        img = np.zeros((self.res_h, self.res_w, 3), dtype=np.uint8)
        
        if self.cam_pose and self.paneles_reales:
            p_cam = np.array([self.cam_pose.pose.position.x, self.cam_pose.pose.position.y, self.cam_pose.pose.position.z])
            r_cam = R.from_quat([self.cam_pose.pose.orientation.x, self.cam_pose.pose.orientation.y, 
                                 self.cam_pose.pose.orientation.z, self.cam_pose.pose.orientation.w]).as_matrix()

            # 1. Proyectar Paneles
            for p in self.paneles_reales:
                pos_p = np.array([p['x'], p['y'], p['z']])
                if np.linalg.norm(pos_p - p_cam) > 150: continue

                ancho, alto = p.get('width_x', 10.4)/2, p.get('length_y', 11.4)/2
                r_p = R.from_euler('xyz', [0, p['pitch'], p['yaw']]).as_matrix()
                
                esquinas_l = [np.array([ancho, alto, 0]), np.array([-ancho, alto, 0]), 
                              np.array([-ancho, -alto, 0]), np.array([ancho, -alto, 0])]
                
                pixels = []
                puntos_fuera = 0
                
                for pt_l in esquinas_l:
                    pt_w = r_p @ pt_l + pos_p
                    u, v, pt_fuera = self.proyectar_a_pixel(pt_w, p_cam, r_cam)
                    pixels.append((u, v))
                    puntos_fuera += pt_fuera 
                
                # LA MAGIA: Si hay menos de 4 puntos fuera (es decir, AL MENOS UNO está dentro)
                if puntos_fuera < 4:
                    # Dibujamos las 4 líneas del rectángulo una por una para evitar errores de OpenCV con los 'None'
                    for i in range(4):
                        p1 = pixels[i]
                        p2 = pixels[(i + 1) % 4]
                        
                        # Solo dibujamos la línea si ambos extremos existen matemáticamente (no están detrás de la cámara)
                        if p1[0] is not None and p2[0] is not None:
                            # OpenCV recorta automáticamente si un punto está fuera de la pantalla (ej: u=800)
                            cv2.line(img, p1, p2, (0, 255, 0), 2)

            # 2. Proyectar Impactos (puntos amarillos)
            for imp in self.impactos_recientes:
                pt_w = np.array(imp.get('rebote_world_debug', [0,0,0]))
                u, v, pt_fuera = self.proyectar_a_pixel(pt_w, p_cam, r_cam)
                
                # Solo dibujamos el impacto si cae DENTRO de la pantalla (pt_fuera == 0)
                if u is not None and pt_fuera == 0:
                    cv2.circle(img, (u, v), 8, (0, 255, 255), -1)

        # Publicar imagen
        self.pub_imagen.publish(self.br.cv2_to_imgmsg(img, encoding="bgr8"))

    def proyectar_a_pixel(self, p_w, p_c, r_c_mat):
        p_local = r_c_mat.T @ (p_w - p_c)
        
        # Corrección 1: Devolver 3 valores siempre
        if p_local[0] <= 0.1: return None, None, 1 
        
        y_p = -self.focal_dist * (p_local[1] / p_local[0])
        z_p = -self.focal_dist * (p_local[2] / p_local[0])
        
        u = int(((y_p / self.sensor_w) + 0.5) * self.res_w)
        v = int(((z_p / self.sensor_h) + 0.5) * self.res_h)
        
        # Corrección 2: Lógica de bordes limpia
        if 0 <= u < self.res_w and 0 <= v < self.res_h:
            return u, v, 0
        return u, v, 1

def main(args=None):
    rclpy.init(args=args)
    nodo = VirtualCameraNode()
    try: rclpy.spin(nodo)
    except KeyboardInterrupt: pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
