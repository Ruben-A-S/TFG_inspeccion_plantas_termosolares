#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray
from scipy.spatial.transform import Rotation as R

class ShowDataNode(Node):
    def __init__(self):
        super().__init__('show_data_node')
        
        # Almacenamiento de estado
        self.dron_pose = None
        self.cam_pose = None
        self.luz_pose = None
        self.reflejos_poses = []

        # --- Parámetros idénticos a area_camera_node.py ---
        self.focal_dist = 1.5  
        self.sensor_w = 1.6    
        self.sensor_h = 1.2    
        self.res_w = 640       
        self.res_h = 480       
        # --------------------------------------------------

        # Suscripciones
        self.create_subscription(PoseStamped, '/datos/dron', self.cb_dron, 10)
        self.create_subscription(PoseStamped, '/datos/camara', self.cb_cam, 10)
        self.create_subscription(PoseStamped, '/datos/luz', self.cb_luz, 10)
        self.create_subscription(PoseArray, '/datos/reflejos', self.cb_reflejos, 10)
        self.create_subscription(PoseArray, '/datos/rebotes', self.cb_rebotes_y_imprimir, 10)

    # Callbacks de guardado
    def cb_dron(self, msg): self.dron_pose = msg
    def cb_cam(self, msg): self.cam_pose = msg
    def cb_luz(self, msg): self.luz_pose = msg
    def cb_reflejos(self, msg): self.reflejos_poses = msg.poses

    def rebote_es_visible(self, p_mundo, p_cam, r_cam_matrix):
        """Aplica el modelo de cámara para saber si el punto cae dentro de la imagen"""
        # 1. Transformar a coordenadas de la cámara
        r_inv = r_cam_matrix.T
        p_c = r_inv @ (p_mundo - p_cam)
        profundidad = p_c[0]

        # Si está detrás de la lente o demasiado cerca
        if profundidad <= 0.1:
            return False

        # 2. Proyectar en el sensor (Modelo Estenopeico)
        y_proj = -self.focal_dist * (p_c[1] / profundidad)
        z_proj = -self.focal_dist * (p_c[2] / profundidad)

        # 3. Convertir a píxeles
        pixel_u = int(((y_proj / self.sensor_w) + 0.5) * self.res_w)
        pixel_v = int(((z_proj / self.sensor_h) + 0.5) * self.res_h)

        # 4. Comprobar si los píxeles están dentro del marco de la imagen
        return (0 <= pixel_u < self.res_w) and (0 <= pixel_v < self.res_h)

    def cb_rebotes_y_imprimir(self, msg_rebotes):
        if not self.dron_pose or not self.cam_pose: return 
        if len(msg_rebotes.poses) != len(self.reflejos_poses): return

        # Extraer variables de la cámara
        c_pos = np.array([self.cam_pose.pose.position.x, self.cam_pose.pose.position.y, self.cam_pose.pose.position.z])
        c_quat = [self.cam_pose.pose.orientation.x, self.cam_pose.pose.orientation.y, self.cam_pose.pose.orientation.z, self.cam_pose.pose.orientation.w]
        r_cam_matrix = R.from_quat(c_quat).as_matrix()

        # Iniciar bloque de consola
        self.get_logger().info("================ DATOS GEOMÉTRICOS ================")
        
        # Dron
        dp = self.dron_pose.pose.position
        do = self.dron_pose.pose.orientation
        self.get_logger().info(f"DRON   -> Pos: ({dp.x:.3f}, {dp.y:.3f}, {dp.z:.3f}) | Ori: ({do.x:.3f}, {do.y:.3f}, {do.z:.3f}, {do.w:.3f})")
        
        # Cámara
        self.get_logger().info(f"CÁMARA -> Pos: ({c_pos[0]:.3f}, {c_pos[1]:.3f}, {c_pos[2]:.3f}) | Ori: ({c_quat[0]:.3f}, {c_quat[1]:.3f}, {c_quat[2]:.3f}, {c_quat[3]:.3f})")
        
        # Filtrado e impresión de rebotes
        rebotes_en_camara = 0
        for i, rebote in enumerate(msg_rebotes.poses):
            r_pos = np.array([rebote.position.x, rebote.position.y, rebote.position.z])
            
            if self.rebote_es_visible(r_pos, c_pos, r_cam_matrix):
                rebotes_en_camara += 1
                self.get_logger().info(f"REBOTE {i} [EN IMAGEN] -> Pos: ({r_pos[0]:.3f}, {r_pos[1]:.3f}, {r_pos[2]:.3f})")
        
        if rebotes_en_camara == 0:
            self.get_logger().info("REBOTES-> [Ningún rebote visible en la matriz de 640x480]")
            
        self.get_logger().info("===================================================\n")

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
