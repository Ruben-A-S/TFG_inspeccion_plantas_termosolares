#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped, PoseArray, Point
from visualization_msgs.msg import Marker, MarkerArray

# --- NUEVO: 1. Librerías para crear y enviar la imagen ---
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
# ---------------------------------------------------------

def vector_a_cuaternion(d_rebote_g):
    norma = np.linalg.norm(d_rebote_g)
    if norma < 1e-6: return [0.0, 0.0, 0.0, 1.0] 
    v_unitario = d_rebote_g / norma
    rotacion, _ = R.align_vectors([v_unitario], [[1.0, 0.0, 0.0]])
    return rotacion.as_quat()

class AreaCameraNode(Node):
    def __init__(self):
        super().__init__('area_camara_node')
        
        self.pub_area_camara = self.create_publisher(MarkerArray, '/visualizacion/area_camara', 10)
        
        # --- NUEVO: 2. Publicador de imagen, puente y resolución ---
        self.pub_imagen = self.create_publisher(Image, '/camara_virtual/imagen', 10)
        self.br = CvBridge()
        self.res_w = 640  # Ancho de la imagen en píxeles
        self.res_h = 480  # Alto de la imagen en píxeles
        # ---------------------------------------------------------
        
        self.dron_pose = None
        self.cam_pose = None
        self.luz_pose = None
        self.paneles_poses = []
        self.reflejos_poses = []

        # Parámetros de la cámara
        self.focal_dist = 1.5  
        self.sensor_w = 1.6    
        self.sensor_h = 1.2    

        self.create_subscription(PoseArray, '/datos/paneles', self.cb_paneles, 10)
        self.create_subscription(PoseStamped, '/datos/dron', self.cb_dron, 10)
        self.create_subscription(PoseStamped, '/datos/camara', self.cb_cam, 10)
        self.create_subscription(PoseStamped, '/datos/luz', self.cb_luz, 10)
        self.create_subscription(PoseArray, '/datos/reflejos', self.cb_reflejos, 10)
        self.create_subscription(PoseArray, '/datos/rebotes', self.cb_rebotes_y_dibujar, 10)

    def cb_paneles(self, msg): self.paneles_poses = msg.poses
    def cb_dron(self, msg): self.dron_pose = msg
    def cb_cam(self, msg): self.cam_pose = msg
    def cb_luz(self, msg): self.luz_pose = msg
    def cb_reflejos(self, msg): self.reflejos_poses = msg.poses

    def proyectar_punto(self, p_mundo, p_cam, r_cam_matrix):
        # 1. Mundo a Cámara
        r_inv = r_cam_matrix.T
        p_c = r_inv @ (p_mundo - p_cam)

        profundidad = p_c[0]

        if profundidad <= 0.1:
            # --- NUEVO: Devolver tupla vacía si no es válido ---
            return None, None, None

        # 2. Modelo Estenopeico (El plano está en -X, y proyectamos Y y Z)
        x_proj = -self.focal_dist
        y_proj = -self.focal_dist * (p_c[1] / profundidad)
        z_proj = -self.focal_dist * (p_c[2] / profundidad)

        p_proj_c = np.array([x_proj, y_proj, z_proj])

        # 3. Cámara a Mundo
        punto_mundo = r_cam_matrix @ p_proj_c + p_cam
        
        # --- NUEVO: 3. Retornamos también y_proj y z_proj para calcular los píxeles ---
        return punto_mundo, y_proj, z_proj
        # -------------------------------------------------------------------------------

    # --- NUEVO: Función auxiliar para convertir metros del sensor a píxeles ---
    def metros_a_pixeles(self, y_proj, z_proj):
        # Mapeamos Y proyectado al eje U (horizontal)
        pixel_u = int(((y_proj / self.sensor_w) + 0.5) * self.res_w)
        # Mapeamos Z proyectado al eje V (vertical). Invertimos el signo porque Z sube, pero V baja en las imágenes.
        pixel_v = int(((z_proj / self.sensor_h) + 0.5) * self.res_h)
        return pixel_u, pixel_v
    # --------------------------------------------------------------------------

    def cb_rebotes_y_dibujar(self, msg_rebotes):
        if not self.dron_pose or not self.luz_pose or not self.cam_pose:
            return 
            
        if len(msg_rebotes.poses) != len(self.reflejos_poses):
            return
            
        marcadores = MarkerArray()
        
        # --- NUEVO: 4. Crear el "lienzo" negro de la imagen ---
        imagen_cv2 = np.zeros((self.res_h, self.res_w, 3), dtype=np.uint8)
        # ------------------------------------------------------
        
        p_cam = np.array([self.cam_pose.pose.position.x, self.cam_pose.pose.position.y, self.cam_pose.pose.position.z])
        q_cam = [self.cam_pose.pose.orientation.x, self.cam_pose.pose.orientation.y, self.cam_pose.pose.orientation.z, self.cam_pose.pose.orientation.w]
        r_cam_matrix = R.from_quat(q_cam).as_matrix()

        esquinas_sensor_c = [
            np.array([-self.focal_dist,  self.sensor_w/2,  self.sensor_h/2]),
            np.array([-self.focal_dist, -self.sensor_w/2,  self.sensor_h/2]),
            np.array([-self.focal_dist, -self.sensor_w/2, -self.sensor_h/2]),
            np.array([-self.focal_dist,  self.sensor_w/2, -self.sensor_h/2]),
            np.array([-self.focal_dist,  self.sensor_w/2,  self.sensor_h/2]) 
        ]
        esquinas_sensor_w = [r_cam_matrix @ pt + p_cam for pt in esquinas_sensor_c]
        marcadores.markers.append(self.crear_linea("sensor_camara", 0, esquinas_sensor_w, color=[1.0, 1.0, 1.0]))

        # --- DIBUJAR PANELES Y SUS PROYECCIONES ---
        for i, pose in enumerate(self.paneles_poses):
            ancho, alto = 1.5, 1.0
            pts_locales = [
                np.array([ancho, alto, 0.0]), np.array([-ancho, alto, 0.0]),
                np.array([-ancho, -alto, 0.0]), np.array([ancho, -alto, 0.0]),
                np.array([ancho, alto, 0.0])
            ]
            
            r_panel = R.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
            p_panel = np.array([pose.position.x, pose.position.y, pose.position.z])
            pts_globales = [r_panel.apply(pt) + p_panel for pt in pts_locales]
            
            marcadores.markers.append(self.crear_linea("esquinas_paneles", i, pts_globales, color=[0.0, 1.0, 0.0]))
            
            pts_proyectados = []
            pixeles_panel = [] # --- NUEVO: Lista para guardar los píxeles de las esquinas
            
            for pt in pts_globales:
                # --- NUEVO: Desempaquetamos los tres valores ---
                proj_mundo, y_p, z_p = self.proyectar_punto(pt, p_cam, r_cam_matrix)
                if proj_mundo is not None:
                    pts_proyectados.append(proj_mundo)
                    # Convertir a píxeles y guardar
                    u, v = self.metros_a_pixeles(y_p, z_p)
                    pixeles_panel.append([u, v])
            
            # Dibujar en RVIZ
            if len(pts_proyectados) > 1: 
                marcadores.markers.append(self.crear_linea("proyeccion_paneles", i, pts_proyectados, color=[0.0, 0.5, 0.0]))
            
            # --- NUEVO: 5. Dibujar el panel en la imagen 2D con OpenCV ---
            if len(pixeles_panel) == 5: # Si se ven todas las esquinas (incluyendo cierre)
                # Convertimos la lista al formato que pide OpenCV
                pts_cv2 = np.array(pixeles_panel, np.int32).reshape((-1, 1, 2))
                # Dibujamos el contorno del panel en verde
                cv2.polylines(imagen_cv2, [pts_cv2], isClosed=True, color=(0, 255, 0), thickness=2)
            # -------------------------------------------------------------

        # --- DIBUJAR LÁSERES, REBOTES Y SUS PROYECCIONES ---
        p_dron = np.array([self.dron_pose.pose.position.x, self.dron_pose.pose.position.y, self.dron_pose.pose.position.z])
        q_dron = [self.dron_pose.pose.orientation.x, self.dron_pose.pose.orientation.y, self.dron_pose.pose.orientation.z, self.dron_pose.pose.orientation.w]
        p_luz = np.array([self.luz_pose.pose.position.x, self.luz_pose.pose.position.y, self.luz_pose.pose.position.z])
        
        marcadores.markers.append(self.crear_flecha("camara", 999, p_cam, q_cam, color=[1.0, 0.0, 0.0]))
        marcadores.markers.append(self.crear_flecha("dron", 999, p_dron, q_dron, color=[1.0, 0.0, 0.0]))
        marcadores.markers.append(self.crear_punto("luz", 1000, p_luz, color=[1.0, 0.0, 0.0]))

        for i, pose_rebote in enumerate(msg_rebotes.poses):
            p_rebote = np.array([pose_rebote.position.x, pose_rebote.position.y, pose_rebote.position.z])
            p_reflejo = np.array([self.reflejos_poses[i].position.x, self.reflejos_poses[i].position.y, self.reflejos_poses[i].position.z])
            
            marcadores.markers.append(self.crear_punto("corte", i, p_rebote, color=[0.0, 1.0, 0.0]))
            marcadores.markers.append(self.crear_punto("reflejo", i, p_reflejo, color=[0.0, 1.0, 0.0]))
            marcadores.markers.append(self.crear_linea("laser_line", i, [p_luz, p_rebote], color=[1.0, 1.0, 0.0]))
            marcadores.markers.append(self.crear_linea("rebote_line", i, [p_rebote, p_dron], color=[1.0, 1.0, 0.0]))
            
            # --- NUEVO: Desempaquetamos los tres valores ---
            proj_rebote_mundo, y_p, z_p = self.proyectar_punto(p_rebote, p_cam, r_cam_matrix)
            if proj_rebote_mundo is not None:
                marcadores.markers.append(self.crear_punto("proyeccion_rebote", i, proj_rebote_mundo, color=[0.0, 1.0, 1.0], scale=0.03))
                
                # --- NUEVO: 6. Dibujar el punto de rebote en la imagen 2D ---
                u, v = self.metros_a_pixeles(y_p, z_p)
                # Si el píxel está dentro de la imagen, dibujamos un círculo amarillo relleno
                if 0 <= u < self.res_w and 0 <= v < self.res_h:
                    cv2.circle(imagen_cv2, (u, v), radius=8, color=(0, 255, 255), thickness=-1)
                # ------------------------------------------------------------

        self.pub_area_camara.publish(marcadores)
        
        # --- NUEVO: 7. Enviar la imagen resultante a ROS ---
        # Pasamos la matriz de numpy (BGR por defecto en OpenCV) a un mensaje tipo sensor_msgs/Image
        mensaje_imagen = self.br.cv2_to_imgmsg(imagen_cv2, encoding="bgr8")
        self.pub_imagen.publish(mensaje_imagen)
        # ---------------------------------------------------

    # --- FUNCIONES DE DIBUJO ---
    def crear_linea(self, ns, m_id, puntos, color=[1.0, 1.0, 0.0]):
        m = Marker()
        m.header.frame_id = "world"
        m.ns, m.id, m.type, m.action = ns, m_id, Marker.LINE_STRIP, Marker.ADD
        m.scale.x = 0.01 
        m.color.r, m.color.g, m.color.b, m.color.a = float(color[0]), float(color[1]), float(color[2]), 1.0
        for p in puntos: m.points.append(Point(x=float(p[0]), y=float(p[1]), z=float(p[2])))
        return m

    def crear_punto(self, ns, m_id, pos, color, scale=0.2):
        m = Marker()
        m.header.frame_id = "world"
        m.ns, m.id, m.type, m.action = ns, m_id, Marker.SPHERE, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(pos[0]), float(pos[1]), float(pos[2])
        m.scale.x, m.scale.y, m.scale.z = scale, scale, scale
        m.color.r, m.color.g, m.color.b, m.color.a = float(color[0]), float(color[1]), float(color[2]), 1.0
        return m

    def crear_flecha(self, ns, m_id, pos, quat, color):
        m = Marker()
        m.header.frame_id = "world"
        m.ns, m.id, m.type, m.action = ns, m_id, Marker.ARROW, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(pos[0]), float(pos[1]), float(pos[2])
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = quat[0], quat[1], quat[2], quat[3]
        m.scale.x, m.scale.y, m.scale.z = 1.0, 0.1, 0.1
        m.color.r, m.color.g, m.color.b, m.color.a = float(color[0]), float(color[1]), float(color[2]), 1.0
        return m

def main(args=None):
    rclpy.init(args=args)
    nodo = AreaCameraNode()
    try: 
        rclpy.spin(nodo)
    except KeyboardInterrupt: 
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
