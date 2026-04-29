#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import String 
import json 

class ShowDataNode(Node):
    def __init__(self):
        super().__init__('show_data_node')
        
        # Almacenamiento de estado
        self.dron_pose = None
        self.cam_pose = None
        
        # Contador para no saturar la terminal propia
        self.contador_frames = 0

        # --- PUBLICADORES ---
        self.pub_feedback = self.create_publisher(String, '/sim_status/log', 10)
        self.pub_datos_normales = self.create_publisher(String, '/inspeccion/datos_normal', 10)

        # --- Parámetros de la cámara ---
        self.focal_dist = 1.5  
        self.sensor_w = 1.6    
        self.sensor_h = 1.2    
        self.res_w = 640       
        self.res_h = 480       
        # -------------------------------

        # --- SUSCRIPCIONES (¡Reducidas!) ---
        self.create_subscription(PoseStamped, '/datos/dron', self.cb_dron, 10)
        self.create_subscription(PoseStamped, '/datos/camara', self.cb_cam, 10)
        self.create_subscription(String, '/inspeccion/datos_crudos', self.cb_datos_crudos, 10)
        
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

    # --- callback UNIFICADO: Hace el feedback visual y el filtrado matemático a la vez ---
    def cb_datos_crudos(self, msg):
        if not self.cam_pose or not self.dron_pose: 
            return

        try:
            datos_crudos = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        # Preparamos las variables de la cámara
        c_pos = np.array([self.cam_pose.pose.position.x, self.cam_pose.pose.position.y, self.cam_pose.pose.position.z])
        c_quat = [self.cam_pose.pose.orientation.x, self.cam_pose.pose.orientation.y, self.cam_pose.pose.orientation.z, self.cam_pose.pose.orientation.w]
        r_cam_matrix = R.from_quat(c_quat).as_matrix()

        paneles_visibles = []
        rebotes_en_camara = 0
        log_text = ""

        # Un solo bucle para todo
        for dato in datos_crudos:
            # --- NUEVO: Extraemos en local y transformamos a global ---
            p_rebote_local = np.array(dato["rebote_local"])
            pos_panel = np.array(dato["pose_panel"]["pos"])
            rot_panel = R.from_quat(dato["pose_panel"]["quat"])
            
            # Pasamos a coordenadas globales para comprobar si la cámara lo ve
            p_rebote_global = pos_panel + rot_panel.apply(p_rebote_local)
            # ----------------------------------------------------------
            
            # Pasamos el filtro mágico usando las coordenadas globales
            if self.rebote_es_visible(p_rebote_global, c_pos, r_cam_matrix):
                paneles_visibles.append(dato) # Ojo: Pasamos el dato original intacto (con sus coordenadas locales) al nodo matemático
                rebotes_en_camara += 1
                # Usamos p_rebote_global para el log, para saber dónde está en el mundo
                log_text += f" | {dato['id_panel']} XYZ:({p_rebote_local[0]:.1f}, {p_rebote_local[1]:.1f}, {p_rebote_local[2]:.1f})"

        # Lógica de publicación unificada
        self.contador_frames += 1

        if rebotes_en_camara > 0:
            # 1. Log en consola propia
            self.get_logger().info(f"¡DESTELLO EN CÁMARA! -> {rebotes_en_camara} paneles detectados.")
            
            # 2. Aviso al nodo de feedback (Colores)
            msg_f = String()
            msg_f.data = f"¡DESTELLO DETECTADO! Impactos: {rebotes_en_camara} {log_text}"
            self.pub_feedback.publish(msg_f)
            
            # 3. Envío de datos puros para el futuro cálculo matemático
            msg_final = String()
            msg_final.data = json.dumps(paneles_visibles)
            self.pub_datos_normales.publish(msg_final)

        elif self.contador_frames % 30 == 0:
            self.get_logger().info("[Analizando]... Ningún láser en el campo de visión.")
    # ---------------------------------------------------------------------------------

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
