#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import String # <-- Añadido para el publicador de feedback

class ShowDataNode(Node):
    def __init__(self):
        super().__init__('show_data_node')
        
        # Almacenamiento de estado
        self.dron_pose = None
        self.cam_pose = None
        
        # Contador para no saturar la terminal propia
        self.contador_frames = 0

        # --- PUBLICADOR PARA EL NODO DE FEEDBACK ---
        self.pub_feedback = self.create_publisher(String, '/sim_status/log', 10)

        # --- Parámetros idénticos a area_camera_node.py ---
        self.focal_dist = 1.5  
        self.sensor_w = 1.6    
        self.sensor_h = 1.2    
        self.res_w = 640       
        self.res_h = 480       
        # --------------------------------------------------

        # Suscripciones (Quitamos Luz y Reflejos porque no hacen falta para la lógica de cámara)
        self.create_subscription(PoseStamped, '/datos/dron', self.cb_dron, 10)
        self.create_subscription(PoseStamped, '/datos/camara', self.cb_cam, 10)
        self.create_subscription(PoseArray, '/datos/rebotes', self.cb_rebotes, 10)

    # Callbacks de guardado
    def cb_dron(self, msg): self.dron_pose = msg
    def cb_cam(self, msg): self.cam_pose = msg

    def rebote_es_visible(self, p_mundo, p_cam, r_cam_matrix):
        """Aplica el modelo de cámara para saber si el punto cae dentro de la imagen"""
        r_inv = r_cam_matrix.T
        p_c = r_inv @ (p_mundo - p_cam)
        profundidad = p_c[0]

        if profundidad <= 0.1:
            return False

        y_proj = -self.focal_dist * (p_c[1] / profundidad)
        z_proj = -self.focal_dist * (p_c[2] / profundidad)

        pixel_u = int(((y_proj / self.sensor_w) + 0.5) * self.res_w)
        pixel_v = int(((z_proj / self.sensor_h) + 0.5) * self.res_h)

        return (0 <= pixel_u < self.res_w) and (0 <= pixel_v < self.res_h)

    def cb_rebotes(self, msg_rebotes):
        if not self.dron_pose or not self.cam_pose: return 

        # Extraer variables de la cámara
        c_pos = np.array([self.cam_pose.pose.position.x, self.cam_pose.pose.position.y, self.cam_pose.pose.position.z])
        c_quat = [self.cam_pose.pose.orientation.x, self.cam_pose.pose.orientation.y, self.cam_pose.pose.orientation.z, self.cam_pose.pose.orientation.w]
        r_cam_matrix = R.from_quat(c_quat).as_matrix()

        rebotes_en_camara = 0
        log_text = ""

        for i, rebote in enumerate(msg_rebotes.poses):
            r_pos = np.array([rebote.position.x, rebote.position.y, rebote.position.z])
            
            if self.rebote_es_visible(r_pos, c_pos, r_cam_matrix):
                rebotes_en_camara += 1
                log_text += f" | P{i} XYZ:({r_pos[0]:.1f}, {r_pos[1]:.1f}, {r_pos[2]:.1f})"
        
        # --- Lógica de impresión y envío al FeedbackNode ---
        self.contador_frames += 1
        
        if rebotes_en_camara > 0:
            # 1. Lo imprimimos en nuestra consola
            self.get_logger().info(f"¡DESTELLO EN CÁMARA! -> {rebotes_en_camara} puntos detectados.")
            
            # 2. Lo enviamos al nodo de Feedback para que avise al usuario
            msg_f = String()
            msg_f.data = f"¡DESTELLO DETECTADO! Impactos: {rebotes_en_camara} {log_text}"
            self.pub_feedback.publish(msg_f)
            
        elif self.contador_frames % 30 == 0:
            # Silencio de radio, solo avisamos localmente de vez en cuando
            self.get_logger().info("[Analizando]... Ningún láser en el campo de visión.")

def main(args=None):
    rclpy.init(args=args)
    nodo = ShowDataNode()
    try: 
        rclpy.spin(nodo)
    except KeyboardInterrupt: 
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
