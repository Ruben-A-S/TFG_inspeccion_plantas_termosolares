#!/usr/bin/env python3

import json
import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_sensor_data

class CameraFilterNode(Node):
    """
    Nodo de Filtrado Óptico 2D (Optimizado con NumPy Vectorial).
    Aplica el modelo de cámara estenopeico a múltiples puntos de golpe.
    """
    def __init__(self):
        super().__init__('camera_filter_node')
        
        # Parámetros de cámara
        self.focal_dist = 1.5
        self.sensor_w, self.sensor_h = 1.6, 1.2
        self.res_w, self.res_h = 640, 480

        self.cam_pose = None

        # --- SUSCRIPCIONES ---
        self.create_subscription(PoseStamped, '/data/camera', self.camara_callback, qos_profile_sensor_data)
        self.create_subscription(String, '/inspection/raw_data', self.datos_crudos_callback, qos_profile_sensor_data)
        
        # --- PUBLICADORES ---
        self.pub_filtered = self.create_publisher(String, '/inspection/filtered_data', 10)
        self.pub_log = self.create_publisher(String, '/sim_status/log', 10)

        self.get_logger().info("Filtro de Cámara Vectorizado activo. Esperando detecciones...")

    def camara_callback(self, msg):
        self.cam_pose = msg

    def datos_crudos_callback(self, msg):
        if not self.cam_pose: return

        try:
            impactos = json.loads(msg.data)
            if not impactos: return
        except:
            return

        # 1. Extraemos la cinemática de la cámara (Usamos la Inversa al igual que en la Virtual Camera)
        p_c = np.array([self.cam_pose.pose.position.x, self.cam_pose.pose.position.y, self.cam_pose.pose.position.z])
        q_cam = self.cam_pose.pose.orientation
        r_cam_inv = R.from_quat([q_cam.x, q_cam.y, q_cam.z, q_cam.w]).inv()

        # 2. VECTORIZACIÓN: Agrupamos todos los impactos
        puntos_mundo = np.array([imp.get('rebote_world_debug', [0, 0, 0]) for imp in impactos])
        
        # Transformación geométrica usando la función apply segura de Scipy
        puntos_cam = r_cam_inv.apply(puntos_mundo - p_c)

        # 3. PROYECCIÓN MASIVA A PÍXELES
        # Máscara 1: Solo puntos que estén delante de la lente (X > 0.1)
        frente_mask = puntos_cam[:, 0] > 0.1
        
        visibles = []
        
        # Si hay al menos 1 punto delante, aplicamos la lente pinhole
        if np.any(frente_mask):
            valid_indices = np.where(frente_mask)[0]
            valid_p = puntos_cam[valid_indices]

            # Ecuaciones estenopeicas calculadas de golpe para el array completo
            y_p = -self.focal_dist * (valid_p[:, 1] / valid_p[:, 0])
            z_p = -self.focal_dist * (valid_p[:, 2] / valid_p[:, 0])

            u = ((y_p / self.sensor_w) + 0.5) * self.res_w
            v = ((z_p / self.sensor_h) + 0.5) * self.res_h

            # Máscara 2: Límites de la pantalla (640x480)
            screen_mask = (u >= 0) & (u < self.res_w) & (v >= 0) & (v < self.res_h)

            # Filtramos los índices finales que sobrevivieron a ambas máscaras
            final_indices = valid_indices[screen_mask]
            visibles = [impactos[i] for i in final_indices]

        # 4. PUBLICACIÓN Y LOG
        if visibles:
            msg_final = String()
            msg_final.data = json.dumps(visibles)
            self.pub_filtered.publish(msg_final)
            
            ids_vistos = list(set([imp.get('id_panel', 'desconocido') for imp in visibles]))
            nombres = ", ".join(ids_vistos)
            
            log = String()
            log.data = f"[FILTER] {len(visibles)} impactos en FOV. Piezas: {nombres}"
            self.pub_log.publish(log)

def main(args=None):
    rclpy.init(args=args)
    nodo = CameraFilterNode()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
