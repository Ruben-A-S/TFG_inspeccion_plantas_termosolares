#!/usr/bin/env python3

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseArray, PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from visualization_msgs.msg import Marker, MarkerArray


def vector_a_cuaternion(d_rebote_g):
    """
    Convierte un vector direccional en el espacio a un cuaternión 
    alineado con el eje X (necesario para las flechas en Rviz).
    """
    norma = np.linalg.norm(d_rebote_g)
    
    if norma < 1e-6: 
        return [0.0, 0.0, 0.0, 1.0] 
        
    v_unitario = d_rebote_g / norma
    rotacion, _ = R.align_vectors([v_unitario], [[1.0, 0.0, 0.0]])
    return rotacion.as_quat()


class RvizVisualizerNode(Node):
    """
    Nodo encargado de transformar las poses matemáticas crudas en
    marcadores visuales (MarkerArray) para ser representados en Rviz.
    """

    def __init__(self):
        super().__init__('rviz_visualizer_node')
        
        self.pub_marcadores = self.create_publisher(
            MarkerArray, '/visualizacion/escena', 10
        )
        
        # Almacenamiento de estado
        self.dron_pose = None
        self.cam_pose = None
        self.luz_pose = None
        self.paneles_poses = []
        self.reflejos_poses = []

        # Suscripciones a la calculadora
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
        
        # El rebote es el que dispara el dibujo (porque es el último dato calculado)
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
    # LÓGICA DE DIBUJO (Disparada por los rebotes)
    # ==========================================

    def rebotes_y_dibujar_callback(self, msg_rebotes):
        """Genera el MarkerArray completo y lo publica para Rviz."""
        if not self.dron_pose or not self.luz_pose:
            self.get_logger().info('Esperando datos del dron o la luz...')
            return 
            
        if len(msg_rebotes.poses) != len(self.reflejos_poses):
            self.get_logger().info(
                f'Desincronización: {len(msg_rebotes.poses)} rebotes vs '
                f'{len(self.reflejos_poses)} reflejos'
            )
            return

        marcadores = MarkerArray()
        stamp_actual = self.get_clock().now().to_msg()
        
        # --- DIBUJAR PANELES ---
        for i, pose in enumerate(self.paneles_poses):
            panel = Marker()
            panel.header.frame_id = "world"
            panel.header.stamp = stamp_actual 
            panel.ns = "esquinas_paneles"
            panel.id = i
            
            panel.type = Marker.LINE_STRIP
            panel.action = Marker.ADD
            panel.pose = pose
            
            panel.scale.x = 0.05  # Grosor del borde
            panel.color.r = 0.0
            panel.color.g = 1.0
            panel.color.b = 0.0
            panel.color.a = 1.0
            
            ancho = 1.5 
            alto = 1.0
            
            # 5 puntos para cerrar el cuadrado (el último vuelve al inicio)
            panel.points = [
                Point(x=ancho, y=alto, z=0.0), 
                Point(x=-ancho, y=alto, z=0.0), 
                Point(x=-ancho, y=-alto, z=0.0), 
                Point(x=ancho, y=-alto, z=0.0),
                Point(x=ancho, y=alto, z=0.0)
            ]
            marcadores.markers.append(panel)

        # --- DIBUJAR DRON, CÁMARA Y LUZ ---
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
        
        p_luz = np.array([
            self.luz_pose.pose.position.x, 
            self.luz_pose.pose.position.y, 
            self.luz_pose.pose.position.z
        ])
        
        marcadores.markers.append(
            self.crear_flecha("camara", 999, p_cam, q_cam, color=[1.0, 0.0, 0.0])
        )
        marcadores.markers.append(
            self.crear_flecha("dron", 998, p_dron, q_dron, color=[1.0, 0.0, 0.0])
        )
        marcadores.markers.append(
            self.crear_punto("luz", 1000, p_luz, color=[1.0, 0.0, 0.0])
        )

        # --- DIBUJAR LÁSERES Y REBOTES ---
        for i, pose_rebote in enumerate(msg_rebotes.poses):
            p_rebote = np.array([
                pose_rebote.position.x, 
                pose_rebote.position.y, 
                pose_rebote.position.z
            ])
            p_reflejo = np.array([
                self.reflejos_poses[i].position.x, 
                self.reflejos_poses[i].position.y, 
                self.reflejos_poses[i].position.z
            ])
            
            # Puntos y reflejos
            marcadores.markers.append(
                self.crear_punto("corte", i, p_rebote, color=[0.0, 1.0, 0.0])
            )
            marcadores.markers.append(
                self.crear_punto("reflejo", i, p_reflejo, color=[0.0, 1.0, 0.0])
            )
            
            # Líneas
            marcadores.markers.append(
                self.crear_linea("laser_line", i, [p_luz, p_rebote])
            )
            marcadores.markers.append(
                self.crear_linea("rebote_line", i, [p_rebote, p_dron])
            )
            
            # Flechas direccionales
            q_laser = vector_a_cuaternion(p_rebote - p_luz)
            marcadores.markers.append(
                self.crear_flecha("laser_dir", i, p_luz, q_laser, color=[1.0, 1.0, 0.0])
            )
            
            q_rebote = vector_a_cuaternion(p_dron - p_rebote)
            marcadores.markers.append(
                self.crear_flecha("rebote_dir", i, p_rebote, q_rebote, color=[0.0, 1.0, 0.0])
            )

        self.pub_marcadores.publish(marcadores)

    # ==========================================
    # FUNCIONES DE CREACIÓN DE MARCADORES
    # ==========================================

    def crear_linea(self, ns, m_id, puntos):
        """Genera un marcador de tipo línea entre varios puntos."""
        m = Marker()
        m.header.frame_id = "world"
        m.ns = ns
        m.id = m_id
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.lifetime.nanosec = 500000000  # 0.5 segundos de vida
        
        m.scale.x = 0.05 
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 0.0
        m.color.a = 1.0
        
        for p in puntos: 
            m.points.append(Point(x=float(p[0]), y=float(p[1]), z=float(p[2])))
            
        return m

    def crear_punto(self, ns, m_id, pos, color):
        """Genera un marcador de tipo esfera en una posición dada."""
        m = Marker()
        m.header.frame_id = "world"
        m.ns = ns
        m.id = m_id
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.lifetime.nanosec = 500000000
        
        m.pose.position.x = pos[0]
        m.pose.position.y = pos[1]
        m.pose.position.z = pos[2]
        
        m.scale.x = 0.2
        m.scale.y = 0.2
        m.scale.z = 0.2
        
        m.color.r = color[0]
        m.color.g = color[1]
        m.color.b = color[2]
        m.color.a = 1.0
        
        return m

    def crear_flecha(self, ns, m_id, pos, quat, color):
        """Genera un marcador de tipo flecha indicando dirección."""
        m = Marker()
        m.header.frame_id = "world"
        m.ns = ns
        m.id = m_id
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.lifetime.nanosec = 500000000
        
        m.pose.position.x = pos[0]
        m.pose.position.y = pos[1]
        m.pose.position.z = pos[2]
        m.pose.orientation.x = quat[0]
        m.pose.orientation.y = quat[1]
        m.pose.orientation.z = quat[2]
        m.pose.orientation.w = quat[3]
        
        m.scale.x = 1.0
        m.scale.y = 0.1
        m.scale.z = 0.1
        
        m.color.r = color[0]
        m.color.g = color[1]
        m.color.b = color[2]
        m.color.a = 1.0
        
        return m


def main(args=None):
    rclpy.init(args=args)
    nodo = RvizVisualizerNode()
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
