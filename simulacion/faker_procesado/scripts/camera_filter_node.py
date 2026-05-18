#!/usr/bin/env python3

import json
import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

class CameraFilterNode(Node):
    """
    Nodo de Filtrado: Actúa como el 'trigger' de inspección.
    Si un impacto es visible por la cámara, lo envía al cerebro.
    """

    def __init__(self):
        super().__init__('camera_filter_node')
        
        # Parámetros de cámara (Deben coincidir con VirtualCameraNode)
        self.focal_dist = 1.5
        self.sensor_w, self.sensor_h = 1.6, 1.2
        self.res_w, self.res_h = 640, 480

        self.cam_pose = None

        # --- SUSCRIPCIONES ---
        self.create_subscription(PoseStamped, '/data/camera', self.camara_callback, 10)
        self.create_subscription(String, '/inspection/raw_data', self.datos_crudos_callback, 10)

        # --- PUBLICADORES ---
        self.pub_filtered = self.create_publisher(String, '/inspection/filtered_data', 10)
        self.pub_log = self.create_publisher(String, '/sim_status/log', 10)

        self.get_logger().info("Filtro de Cámara activo. Esperando detecciones...")

    def camara_callback(self, msg):
        self.cam_pose = msg

    def datos_crudos_callback(self, msg):
        """
        Recibe los impactos calculados por el Faker.
        Usa la pose de la cámara para filtrar cuáles son visibles.
        """
        if not self.cam_pose: return

        try:
            impactos_raw = json.loads(msg.data)
        except:
            return

        visibles = []
        
        # Extraer matrices de cámara una sola vez por frame
        p_c = np.array([self.cam_pose.pose.position.x, self.cam_pose.pose.position.y, self.cam_pose.pose.position.z])
        r_c_mat = R.from_quat([self.cam_pose.pose.orientation.x, self.cam_pose.pose.orientation.y, 
                               self.cam_pose.pose.orientation.z, self.cam_pose.pose.orientation.w]).as_matrix()

        for imp in impactos_raw:
            # IMPORTANTE: El Faker ahora nos da el impacto en coordenadas MUNDO para facilitar esto
            # o podemos usar la pose del panel que el Faker también conoce.
            # Suponiendo que el Faker envía 'rebote_world'
            p_w = np.array(imp.get('rebote_world_debug', [0,0,0]))
            
            if self.punto_en_fov(p_w, p_c, r_c_mat):
                visibles.append(imp)

        if visibles:
            # Enviar al Cerebro
            msg_final = String()
            msg_final.data = json.dumps(visibles)
            self.pub_filtered.publish(msg_final)
            
            # Log de sistema
            log = String()
            log.data = f"[FILTER] {len(visibles)} impactos detectados en FOV."
            self.pub_log.publish(log)

    def punto_en_fov(self, p_mundo, p_cam, r_cam_matrix):
        """Modelo estenopeico para validación de visibilidad."""
        p_c = r_cam_matrix.T @ (p_mundo - p_cam)
        
        if p_c[0] <= 0.1: return False # Detrás de la cámara
        
        # Proyección
        y_p = -self.focal_dist * (p_c[1] / p_c[0])
        z_p = -self.focal_dist * (p_c[2] / p_c[0])

        # Mapeo a píxeles
        u = int(((y_p / self.sensor_w) + 0.5) * self.res_w)
        v = int(((z_p / self.sensor_h) + 0.5) * self.res_h)

        return (0 <= u < self.res_w) and (0 <= v < self.res_h)

def main(args=None):
    rclpy.init(args=args)
    nodo = CameraFilterNode()
    rclpy.spin(nodo)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
