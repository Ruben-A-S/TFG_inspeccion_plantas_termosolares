#!/usr/bin/env python3

import json

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import String


class CameraFilterNode(Node):
    """
    Nodo encargado de filtrar los datos matemáticos crudos.
    
    Comprueba si los rebotes teóricos calculados caen dentro del campo 
    de visión de la cámara virtual. Si es así, publica los datos para 
    su procesamiento final y avisa por la terminal de logs.
    """

    def __init__(self):
        super().__init__('camera_filter_node')
        
        # Almacenamiento de estado
        self.dron_pose = None
        self.cam_pose = None
        
        # Contador para no saturar la terminal propia
        self.contador_frames = 0

        # --- PUBLICADORES ---
        self.pub_feedback = self.create_publisher(
            String, '/sim_status/log', 10
        )
        self.pub_datos_normales = self.create_publisher(
            String, '/inspeccion/datos_filtrados', 10
        )

        # --- PARÁMETROS DE LA CÁMARA ---
        self.focal_dist = 1.5  
        self.sensor_w = 1.6    
        self.sensor_h = 1.2    
        self.res_w = 640       
        self.res_h = 480       

        # --- SUSCRIPCIONES ---
        self.create_subscription(
            PoseStamped, '/datos/dron', self.dron_callback, 10
        )
        self.create_subscription(
            PoseStamped, '/datos/camara', self.camara_callback, 10
        )
        self.create_subscription(
            String, '/inspeccion/datos_crudos', self.datos_crudos_callback, 10
        )

    # ==========================================
    # CALLBACKS DE GUARDADO DE ESTADO
    # ==========================================

    def dron_callback(self, msg): 
        self.dron_pose = msg

    def camara_callback(self, msg): 
        self.cam_pose = msg

    # ==========================================
    # LÓGICA DE FILTRADO
    # ==========================================

    def rebote_es_visible(self, p_mundo, p_cam, r_cam_matrix):
        """
        Aplica el modelo de cámara estenopeica para saber si 
        un punto 3D cae dentro del sensor de imagen 2D.
        """
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

    def datos_crudos_callback(self, msg):
        """
        Recibe los cálculos brutos de todos los paneles, filtra los que no
        están en cámara, y re-publica solo los válidos.
        """
        if not self.cam_pose or not self.dron_pose: 
            return

        try:
            datos_crudos = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        # Preparamos las variables de la cámara
        c_pos = np.array([
            self.cam_pose.pose.position.x, 
            self.cam_pose.pose.position.y, 
            self.cam_pose.pose.position.z
        ])
        
        c_quat = [
            self.cam_pose.pose.orientation.x, 
            self.cam_pose.pose.orientation.y, 
            self.cam_pose.pose.orientation.z, 
            self.cam_pose.pose.orientation.w
        ]
        r_cam_matrix = R.from_quat(c_quat).as_matrix()

        paneles_visibles = []
        rebotes_en_camara = 0
        log_text = ""

        # Un solo bucle para todo
        for dato in datos_crudos:
            p_rebote_local = np.array(dato["rebote_local"])
            pos_panel = np.array(dato["pose_panel"]["pos"])
            rot_panel = R.from_quat(dato["pose_panel"]["quat"])
            
            # Pasamos a coordenadas globales para comprobar si la cámara lo ve
            p_rebote_global = pos_panel + rot_panel.apply(p_rebote_local)
            
            # Pasamos el filtro mágico usando las coordenadas globales
            if self.rebote_es_visible(p_rebote_global, c_pos, r_cam_matrix):
                # Guardamos el dato original intacto (con sus coordenadas locales)
                paneles_visibles.append(dato) 
                rebotes_en_camara += 1
                
                # Usamos las locales para el log, para saber dónde está en el panel
                log_text += (
                    f" | {dato['id_panel']} XYZ:"
                    f"({p_rebote_local[0]:.1f}, {p_rebote_local[1]:.1f}, "
                    f"{p_rebote_local[2]:.1f})"
                )

        # Lógica de publicación unificada
        self.contador_frames += 1

        if rebotes_en_camara > 0:
            # 1. Log en consola propia
            self.get_logger().info(
                f"¡DESTELLO EN CÁMARA! -> {rebotes_en_camara} paneles detectados."
            )
            
            # 2. Aviso al nodo de feedback (Cliente/Servidor)
            msg_f = String()
            msg_f.data = f"¡DESTELLO DETECTADO! Impactos: {rebotes_en_camara} {log_text}"
            self.pub_feedback.publish(msg_f)
            
            # 3. Envío de datos filtrados para el cálculo final
            msg_final = String()
            msg_final.data = json.dumps(paneles_visibles)
            self.pub_datos_normales.publish(msg_final)

        elif self.contador_frames % 30 == 0:
            self.get_logger().info("[Analizando]... Ningún láser en el campo de visión.")


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
