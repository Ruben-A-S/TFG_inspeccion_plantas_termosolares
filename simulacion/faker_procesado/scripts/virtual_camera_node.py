#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
import json
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from scipy.spatial import KDTree
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rclpy.qos import qos_profile_sensor_data

class VirtualCameraNode(Node):
    def __init__(self):
        super().__init__('virtual_camera_node')
        
        self.br = CvBridge()
        self.res_w, self.res_h = 640, 480
        self.focal_dist = 1.5
        self.sensor_w, self.sensor_h = 1.6, 1.2
        
        self.limite_coseno_fov = np.cos(np.radians(160.0 / 2.0))
        self.distancia_dibujo = 150.0
        
        self.dron_pose = None
        self.cam_pose = None
        self.impactos_recientes = []
        
        self.paneles_memoria = []
        self.coords_paneles = []
        self.arbol_kd = None

        self.cli_realidad = self.create_client(Trigger, 'get_panel_real')

        self.pub_imagen = self.create_publisher(Image, '/virtual_camera/image', 10)
        self.create_subscription(String, '/sim_status/panel_updates', self.actualizar_paneles_callback, 10)
        self.create_subscription(PoseStamped, '/data/drone', self.dron_callback, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/data/camera', self.camara_callback, qos_profile_sensor_data)
        self.create_subscription(String, '/inspection/raw_data', self.raw_data_callback, qos_profile_sensor_data)
        
        self.create_timer(0.05, self.render_loop)
        self.pedir_mapa_inicial()
        self.get_logger().info("Cámara Virtual [KD-Tree + RAM Pura] activa.")

    def pedir_mapa_inicial(self):
        if not self.cli_realidad.service_is_ready(): return
        req = Trigger.Request()
        self.cli_realidad.call_async(req).add_done_callback(self.al_recibir_mapa_srv)

    def al_recibir_mapa_srv(self, futuro):
        try:
            res = futuro.result()
            self.paneles_memoria = json.loads(res.message)
            self.coords_paneles = [[p['x'], p['y'], p['z']] for p in self.paneles_memoria]
            if self.coords_paneles:
                self.arbol_kd = KDTree(self.coords_paneles)
        except Exception as e: self.get_logger().error(f"Error mapa: {e}")

    def actualizar_paneles_callback(self, msg):
        self.pedir_mapa_inicial()

    def dron_callback(self, msg): self.dron_pose = msg
    def camara_callback(self, msg): self.cam_pose = msg
    def raw_data_callback(self, msg):
        try: self.impactos_recientes = json.loads(msg.data)
        except: self.impactos_recientes = []

    def render_loop(self):
        img = np.zeros((self.res_h, self.res_w, 3), dtype=np.uint8)
        if not self.cam_pose or not self.arbol_kd:
            self.pub_imagen.publish(self.br.cv2_to_imgmsg(img, encoding="bgr8"))
            return

        p_cam = np.array([self.cam_pose.pose.position.x, self.cam_pose.pose.position.y, self.cam_pose.pose.position.z])
        q_cam = self.cam_pose.pose.orientation
        r_cam_inv = R.from_quat([q_cam.x, q_cam.y, q_cam.z, q_cam.w]).inv()
        eje_optico = R.from_quat([q_cam.x, q_cam.y, q_cam.z, q_cam.w]).apply([1, 0, 0])

        indices_cercanos = self.arbol_kd.query_ball_point(p_cam, r=self.distancia_dibujo)

        for idx in indices_cercanos:
            p = self.paneles_memoria[idx] 
            pos_p_global = np.array([p['x'], p['y'], p['z']])
            
            vec_dir = pos_p_global - p_cam
            dist = np.linalg.norm(vec_dir)
            if dist < 1e-3: continue
            if np.dot(eje_optico, vec_dir/dist) < self.limite_coseno_fov: continue 

            # TU CINEMÁTICA ORIGINAL (Síncrona)
            rot_yaw = R.from_euler('z', p.get('yaw', 0.0))
            rot_pitch = R.from_euler('y', p.get('pitch', 0.0))
            r_p_global = rot_yaw * rot_pitch

            objetivos_dibujo = []
            if 'facetas' in p:
                w_f, l_f = p.get('width_x', 10.4) / 5.0, p.get('length_y', 11.4) / 5.0
                for f in p['facetas']:
                    offset_local = np.array(f.get('offset', [0.0, 0.0, 0.0]))
                    pos_f_global = pos_p_global + r_p_global.apply(offset_local)
                    
                    rot_canting = R.from_euler('xy', [f.get('cant_roll', 0.0), f.get('cant_pitch', 0.0)])
                    r_f_final = r_p_global * rot_canting
                    
                    objetivos_dibujo.append({'pos': pos_f_global, 'rot_mat': r_f_final, 'w': w_f, 'l': l_f})
            else:
                objetivos_dibujo.append({'pos': pos_p_global, 'rot_mat': r_p_global, 'w': p.get('width_x', 10.4), 'l': p.get('length_y', 11.4)})

            for obj in objetivos_dibujo:
                hw, hl = obj['w']/2, obj['l']/2
                esquinas_locales = np.array([[hw, hl, 0], [-hw, hl, 0], [-hw, -hl, 0], [hw, -hl, 0]])
                esquinas_mundo = obj['rot_mat'].apply(esquinas_locales) + obj['pos']
                esquinas_cam = r_cam_inv.apply(esquinas_mundo - p_cam)
                
                # Proyección vectorizada Anti-Parpadeo
                if np.all(esquinas_cam[:, 0] > 0.1):
                    u = (((-self.focal_dist * (esquinas_cam[:, 1] / esquinas_cam[:, 0])) / self.sensor_w) + 0.5) * self.res_w
                    v = (((-self.focal_dist * (esquinas_cam[:, 2] / esquinas_cam[:, 0])) / self.sensor_h) + 0.5) * self.res_h
                    
                    pts = np.vstack((u, v)).T.astype(np.int32)
                    cv2.polylines(img, [pts], isClosed=True, color=(0, 255, 0), thickness=2)

        if self.impactos_recientes:
            pts_mundo = np.array([imp.get('rebote_world_debug', [0,0,0]) for imp in self.impactos_recientes])
            pts_cam = r_cam_inv.apply(pts_mundo - p_cam)
            frente_mask = pts_cam[:, 0] > 0.1
            if np.any(frente_mask):
                valid_p = pts_cam[frente_mask]
                u = (((-self.focal_dist * (valid_p[:, 1] / valid_p[:, 0])) / self.sensor_w) + 0.5) * self.res_w
                v = (((-self.focal_dist * (valid_p[:, 2] / valid_p[:, 0])) / self.sensor_h) + 0.5) * self.res_h
                dentro_mask = (u >= 0) & (u < self.res_w) & (v >= 0) & (v < self.res_h)
                for ui, vi in zip(u[dentro_mask], v[dentro_mask]):
                    cv2.circle(img, (int(ui), int(vi)), 8, (0, 255, 255), -1)

        self.pub_imagen.publish(self.br.cv2_to_imgmsg(img, encoding="bgr8"))

def main(args=None):
    rclpy.init(args=args)
    nodo = VirtualCameraNode()
    try: rclpy.spin(nodo)
    except KeyboardInterrupt: pass
    finally: nodo.destroy_node(); rclpy.shutdown()

if __name__ == "__main__": main()
