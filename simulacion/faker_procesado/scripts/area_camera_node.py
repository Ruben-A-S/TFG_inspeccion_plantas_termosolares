#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PoseArray, PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray


def vector_a_cuaternion(d_rebote_g):
    """Convierte un vector direccional a un cuaternión alineado con X."""
    norma = np.linalg.norm(d_rebote_g)
    
    if norma < 1e-6: 
        return [0.0, 0.0, 0.0, 1.0] 
        
    v_unitario = d_rebote_g / norma
    rotacion, _ = R.align_vectors([v_unitario], [[1.0, 0.0, 0.0]])
    return rotacion.as_quat()


class VirtualCameraNode(Node):
    """
    Nodo que genera una cámara virtual proyectando puntos 3D del entorno
    en un plano 2D usando el modelo de cámara estenopeica (pinhole).
    """

    def __init__(self):
        super().__init__('virtual_camera_node')
        
        # --- PUBLICADORES ---
        self.pub_area_camara = self.create_publisher(
            MarkerArray, '/visualizacion/area_camara', 10
        )
        self.pub_imagen = self.create_publisher(
            Image, '/camara_virtual/imagen', 10
        )
        
        # --- CONFIGURACIÓN DE CÁMARA Y OPENCV ---
        self.br = CvBridge()
        self.res_w = 640  # Ancho de la imagen en píxeles
        self.res_h = 480  # Alto de la imagen en píxeles
        
        # Parámetros físicos de la cámara
        self.focal_dist = 1.5  
        self.sensor_w = 1.6    
        self.sensor_h = 1.2    
        
        # Estado interno
        self.dron_pose = None
        self.cam_pose = None
        self.luz_pose = None
        self.paneles_poses = []
        self.reflejos_poses = []

        # --- SUSCRIPCIONES ---
        self.create_subscription(
            PoseArray, '/datos/paneles', self.paneles_callback, 10
        )
        self.create_subscription(
            PoseStamped, '/datos/dron', self.dron_callback, 10
        )
        self.create_subscription(
            PoseStamped, '/datos/camara', self.camara_callback, 10
        )
        self.create_subscription(
            PoseStamped, '/datos/luz', self.luz_callback, 10
        )
        self.create_subscription(
            PoseArray, '/datos/reflejos', self.reflejos_callback, 10
        )
        self.create_subscription(
            PoseArray, '/datos/rebotes', self.rebotes_y_dibujar_callback, 10
        )

    # ==========================================
    # CALLBACKS DE GUARDADO DE ESTADO
    # ==========================================
    
    def paneles_callback(self, msg): 
        self.paneles_poses = msg.poses

    def dron_callback(self, msg): 
        self.dron_pose = msg

    def camara_callback(self, msg): 
        self.cam_pose = msg

    def luz_callback(self, msg): 
        self.luz_pose = msg

    def reflejos_callback(self, msg): 
        self.reflejos_poses = msg.poses

    # ==========================================
    # MATEMÁTICAS DE PROYECCIÓN 3D -> 2D
    # ==========================================

    def proyectar_punto(self, p_mundo, p_cam, r_cam_matrix):
        """
        Transforma un punto global 3D a las coordenadas locales de la cámara
        y lo proyecta en el plano del sensor (modelo estenopeico).
        """
        # 1. Transformación de Mundo a Cámara
        r_inv = r_cam_matrix.T
        p_c = r_inv @ (p_mundo - p_cam)

        profundidad = p_c[0]

        # Si el punto está detrás o demasiado cerca de la lente, se descarta
        if profundidad <= 0.1:
            return None, None, None

        # 2. Modelo Estenopeico (El plano está en -X, proyectamos Y y Z)
        x_proj = -self.focal_dist
        y_proj = -self.focal_dist * (p_c[1] / profundidad)
        z_proj = -self.focal_dist * (p_c[2] / profundidad)

        p_proj_c = np.array([x_proj, y_proj, z_proj])

        # 3. De Cámara a Mundo (para visualizar en Rviz)
        punto_mundo = r_cam_matrix @ p_proj_c + p_cam
        
        # Retornamos el punto 3D para Rviz y las coordenadas locales para OpenCV
        return punto_mundo, y_proj, z_proj

    def metros_a_pixeles(self, y_proj, z_proj):
        """Mapea las coordenadas físicas del sensor a píxeles de la imagen."""
        # Mapeamos Y proyectado al eje U (horizontal)
        pixel_u = int(((y_proj / self.sensor_w) + 0.5) * self.res_w)
        
        # Mapeamos Z proyectado al eje V (vertical). 
        # Invertimos el signo porque Z sube, pero V baja en las imágenes.
        pixel_v = int(((z_proj / self.sensor_h) + 0.5) * self.res_h)
        
        return pixel_u, pixel_v

    # ==========================================
    # LÓGICA PRINCIPAL (Disparada por rebotes)
    # ==========================================

    def rebotes_y_dibujar_callback(self, msg_rebotes):
        """Genera los marcadores 3D y la imagen sintética 2D."""
        if not self.dron_pose or not self.luz_pose or not self.cam_pose:
            return 
            
        if len(msg_rebotes.poses) != len(self.reflejos_poses):
            return
            
        marcadores = MarkerArray()
        
        # Crear el "lienzo" negro de la imagen OpenCV (formato BGR)
        imagen_cv2 = np.zeros((self.res_h, self.res_w, 3), dtype=np.uint8)
        
        p_cam = np.array([
            self.cam_pose.pose.position.x, 
            self.cam_pose.pose.position.y, 
            self.cam_pose.pose.position.z
        ])
        q_cam = [
            self.cam_pose.pose.orientation.x, 
            self.cam_pose.pose.orientation.y, 
            self.cam_pose.pose.orientation.z, 
            self.cam_pose.pose.orientation.w
        ]
        r_cam_matrix = R.from_quat(q_cam).as_matrix()

        # --- DIBUJAR MARCO DEL SENSOR EN RVIZ ---
        esquinas_sensor_c = [
            np.array([-self.focal_dist,  self.sensor_w/2,  self.sensor_h/2]),
            np.array([-self.focal_dist, -self.sensor_w/2,  self.sensor_h/2]),
            np.array([-self.focal_dist, -self.sensor_w/2, -self.sensor_h/2]),
            np.array([-self.focal_dist,  self.sensor_w/2, -self.sensor_h/2]),
            np.array([-self.focal_dist,  self.sensor_w/2,  self.sensor_h/2]) 
        ]
        esquinas_sensor_w = [r_cam_matrix @ pt + p_cam for pt in esquinas_sensor_c]
        marcadores.markers.append(
            self.crear_linea("sensor_camara", 0, esquinas_sensor_w, color=[1.0, 1.0, 1.0])
        )

        # --- PROYECTAR PANELES ---
        for i, pose in enumerate(self.paneles_poses):
            ancho = 5.7075
            alto = 5.2105
            pts_locales = [
                np.array([ancho, alto, 0.0]), 
                np.array([-ancho, alto, 0.0]),
                np.array([-ancho, -alto, 0.0]), 
                np.array([ancho, -alto, 0.0]),
                np.array([ancho, alto, 0.0])
            ]
            
            r_panel = R.from_quat([
                pose.orientation.x, pose.orientation.y, 
                pose.orientation.z, pose.orientation.w
            ])
            p_panel = np.array([pose.position.x, pose.position.y, pose.position.z])
            pts_globales = [r_panel.apply(pt) + p_panel for pt in pts_locales]
            
            marcadores.markers.append(
                self.crear_linea("esquinas_paneles", i, pts_globales, color=[0.0, 1.0, 0.0])
            )
            
            pts_proyectados = []
            pixeles_panel = [] 
            
            for pt in pts_globales:
                proj_mundo, y_p, z_p = self.proyectar_punto(pt, p_cam, r_cam_matrix)
                if proj_mundo is not None:
                    pts_proyectados.append(proj_mundo)
                    
                    # Convertir a píxeles y guardar
                    u, v = self.metros_a_pixeles(y_p, z_p)
                    pixeles_panel.append([u, v])
            
            # Dibujar en RVIZ
            if len(pts_proyectados) > 1: 
                marcadores.markers.append(
                    self.crear_linea("proyeccion_paneles", i, pts_proyectados, color=[0.0, 0.5, 0.0])
                )
            
            # Dibujar el panel en la imagen 2D con OpenCV
            if len(pixeles_panel) == 5: 
                pts_cv2 = np.array(pixeles_panel, np.int32).reshape((-1, 1, 2))
                cv2.polylines(
                    imagen_cv2, [pts_cv2], isClosed=True, color=(0, 255, 0), thickness=2
                )

        # --- DIBUJAR LÁSERES, REBOTES Y SUS PROYECCIONES ---
        p_dron = np.array([
            self.dron_pose.pose.position.x, 
            self.dron_pose.pose.position.y, 
            self.dron_pose.pose.position.z
        ])
        q_dron = [
            self.dron_pose.pose.orientation.x, 
            self.dron_pose.pose.orientation.y, 
            self.dron_pose.pose.orientation.z, 
            self.dron_pose.pose.orientation.w
        ]
        p_luz = np.array([
            self.luz_pose.pose.position.x, 
            self.luz_pose.pose.position.y, 
            self.luz_pose.pose.position.z
        ])
        
        marcadores.markers.append(
            self.crear_flecha("camara", 999, p_cam, q_cam, color=[1.0, 0.0, 0.0])
        )
        marcadores.markers.append(
            self.crear_flecha("dron", 999, p_dron, q_dron, color=[1.0, 0.0, 0.0])
        )
        marcadores.markers.append(
            self.crear_punto("luz", 1000, p_luz, color=[1.0, 0.0, 0.0])
        )

        for i, pose_rebote in enumerate(msg_rebotes.poses):
            p_rebote = np.array([
                pose_rebote.position.x, pose_rebote.position.y, pose_rebote.position.z
            ])
            p_reflejo = np.array([
                self.reflejos_poses[i].position.x, 
                self.reflejos_poses[i].position.y, 
                self.reflejos_poses[i].position.z
            ])
            
            marcadores.markers.append(
                self.crear_punto("corte", i, p_rebote, color=[0.0, 1.0, 0.0])
            )
            marcadores.markers.append(
                self.crear_punto("reflejo", i, p_reflejo, color=[0.0, 1.0, 0.0])
            )
            marcadores.markers.append(
                self.crear_linea("laser_line", i, [p_luz, p_rebote], color=[1.0, 1.0, 0.0])
            )
            marcadores.markers.append(
                self.crear_linea("rebote_line", i, [p_rebote, p_dron], color=[1.0, 1.0, 0.0])
            )
            
            # Proyección del rebote en la cámara virtual
            proj_rebote_mundo, y_p, z_p = self.proyectar_punto(p_rebote, p_cam, r_cam_matrix)
            
            if proj_rebote_mundo is not None:
                marcadores.markers.append(
                    self.crear_punto(
                        "proyeccion_rebote", i, proj_rebote_mundo, color=[0.0, 1.0, 1.0], scale=0.03
                    )
                )
                
                # Dibujar el punto de rebote en la imagen 2D
                u, v = self.metros_a_pixeles(y_p, z_p)
                
                if 0 <= u < self.res_w and 0 <= v < self.res_h:
                    cv2.circle(
                        imagen_cv2, (u, v), radius=8, color=(0, 255, 255), thickness=-1
                    )

        # Publicar los marcadores 3D para Rviz
        self.pub_area_camara.publish(marcadores)
        
        # Enviar la imagen resultante a ROS usando CvBridge
        mensaje_imagen = self.br.cv2_to_imgmsg(imagen_cv2, encoding="bgr8")
        self.pub_imagen.publish(mensaje_imagen)

    # ==========================================
    # FUNCIONES DE CREACIÓN DE MARCADORES
    # ==========================================

    def crear_linea(self, ns, m_id, puntos, color=None):
        if color is None:
            color = [1.0, 1.0, 0.0]
            
        m = Marker()
        m.header.frame_id = "world"
        m.ns = ns
        m.id = m_id
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.01 
        
        m.color.r = float(color[0])
        m.color.g = float(color[1])
        m.color.b = float(color[2])
        m.color.a = 1.0
        
        for p in puntos: 
            m.points.append(Point(x=float(p[0]), y=float(p[1]), z=float(p[2])))
            
        return m

    def crear_punto(self, ns, m_id, pos, color, scale=0.2):
        m = Marker()
        m.header.frame_id = "world"
        m.ns = ns
        m.id = m_id
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        
        m.pose.position.x = float(pos[0])
        m.pose.position.y = float(pos[1])
        m.pose.position.z = float(pos[2])
        
        m.scale.x = scale
        m.scale.y = scale
        m.scale.z = scale
        
        m.color.r = float(color[0])
        m.color.g = float(color[1])
        m.color.b = float(color[2])
        m.color.a = 1.0
        
        return m

    def crear_flecha(self, ns, m_id, pos, quat, color):
        m = Marker()
        m.header.frame_id = "world"
        m.ns = ns
        m.id = m_id
        m.type = Marker.ARROW
        m.action = Marker.ADD
        
        m.pose.position.x = float(pos[0])
        m.pose.position.y = float(pos[1])
        m.pose.position.z = float(pos[2])
        
        m.pose.orientation.x = quat[0]
        m.pose.orientation.y = quat[1]
        m.pose.orientation.z = quat[2]
        m.pose.orientation.w = quat[3]
        
        m.scale.x = 1.0
        m.scale.y = 0.1
        m.scale.z = 0.1
        
        m.color.r = float(color[0])
        m.color.g = float(color[1])
        m.color.b = float(color[2])
        m.color.a = 1.0
        
        return m


def main(args=None):
    rclpy.init(args=args)
    nodo = VirtualCameraNode()
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
