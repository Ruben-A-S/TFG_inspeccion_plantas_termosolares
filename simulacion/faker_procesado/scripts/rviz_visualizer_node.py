#!/usr/bin/env python3

import json
import numpy as np
import rclpy
from rclpy.node import Node
# CORRECCIÓN: Faltaba importar String
from std_msgs.msg import String
from geometry_msgs.msg import Point, PoseArray, PoseStamped, Pose
from scipy.spatial.transform import Rotation as R
from visualization_msgs.msg import Marker, MarkerArray

def vector_a_cuaternion(vector_dir):
    """Convierte un vector direccional a un cuaternión alineado con el eje X."""
    norma = np.linalg.norm(vector_dir)
    if norma < 1e-6:
        return [0.0, 0.0, 0.0, 1.0]
    
    v_unitario = vector_dir / norma
    try:
        rotacion, _ = R.align_vectors([v_unitario], [[1.0, 0.0, 0.0]])
        return rotacion.as_quat().tolist()
    except:
        return [0.0, 0.0, 0.0, 1.0]

class RvizVisualizerNode(Node):
    def __init__(self):
        super().__init__('rviz_visualizer_node')
        
        # --- ESTADO INTERNO ---
        self.dron_pose = None
        self.cam_pose = None
        self.luz_pose = None
        self.paneles_poses = []
        self.impactos_data = [] 

        # --- PUBLICADOR ---
        self.pub_marcadores = self.create_publisher(MarkerArray, '/visualization/scene', 10)
        
        # --- SUSCRIPCIONES ---
        self.create_subscription(PoseArray, '/data/panels', self.paneles_callback, 10)
        self.create_subscription(PoseStamped, '/data/drone', self.dron_callback, 10)
        self.create_subscription(PoseStamped, '/data/camera', self.camara_callback, 10)
        self.create_subscription(PoseStamped, '/data/light', self.luz_callback, 10)
        self.create_subscription(String, '/inspection/raw_data', self.raw_data_callback, 10)

        # --- TIMER DE DIBUJO (10 Hz) ---
        self.timer = self.create_timer(0.1, self.publicar_escena)

        self.get_logger().info("Visualizador RViz iniciado correctamente.")

    # ==========================================
    # CALLBACKS
    # ==========================================

    def paneles_callback(self, msg): 
        self.paneles_poses = msg.poses

    def dron_callback(self, msg): 
        self.dron_pose = msg

    def camara_callback(self, msg): 
        self.cam_pose = msg

    def luz_callback(self, msg): 
        self.luz_pose = msg
    
    def raw_data_callback(self, msg):
        try:
            self.impactos_data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"Error parseando JSON: {e}")
            self.impactos_data = []

    # ==========================================
    # BUCLE DE RENDERIZADO
    # ==========================================

    def publicar_escena(self):
        if not self.dron_pose:
            return

        msg_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        # 1. DIBUJAR PANELES
        for i, pose in enumerate(self.paneles_poses):
            msg_array.markers.append(self.crear_marcador_panel(i, pose, stamp))

        # 2. DIBUJAR DRON Y CÁMARA
        p_dron = self.pose_to_numpy(self.dron_pose.pose.position)
        q_dron = self.quat_to_list(self.dron_pose.pose.orientation)
        msg_array.markers.append(self.crear_flecha("dron", 998, p_dron, q_dron, [0.0, 0.5, 1.0], stamp))

        if self.cam_pose:
            q_cam = self.quat_to_list(self.cam_pose.pose.orientation)
            msg_array.markers.append(self.crear_flecha("camara", 999, p_dron, q_cam, [1.0, 0.0, 0.0], stamp))

        # 3. DIBUJAR RAYOS / IMPACTOS
        if self.luz_pose:
            p_luz = self.pose_to_numpy(self.luz_pose.pose.position)
            for i, impacto in enumerate(self.impactos_data):
                p_rebote_world = np.array(impacto.get("rebote_world_debug", [0,0,0]))
                if not np.array_equal(p_rebote_world, [0,0,0]):
                    # Dibujar esfera de impacto
                    m_impacto = Marker()
                    m_impacto.header.frame_id = "world"
                    m_impacto.header.stamp = stamp
                    m_impacto.ns = "impactos"
                    m_impacto.id = i
                    m_impacto.type = Marker.SPHERE
                    m_impacto.pose.position.x, m_impacto.pose.position.y, m_impacto.pose.position.z = p_rebote_world
                    m_impacto.scale.x, m_impacto.scale.y, m_impacto.scale.z = 0.3, 0.3, 0.3
                    m_impacto.color.r, m_impacto.color.g, m_impacto.color.b, m_impacto.color.a = 1.0, 1.0, 0.0, 1.0
                    msg_array.markers.append(m_impacto)

        self.pub_marcadores.publish(msg_array)

    # ==========================================
    # UTILIDADES
    # ==========================================

    def crear_marcador_panel(self, id, pose, stamp):
        m = Marker()
        m.header.frame_id = "world"
        m.header.stamp = stamp
        m.ns = "heliostatos"
        m.id = id
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.pose = pose
        m.scale.x = 0.1 
        m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 1.0
        
        hw, hl = 5.2, 5.7 
        m.points = [
            Point(x=hw, y=hl, z=0.0), Point(x=-hw, y=hl, z=0.0),
            Point(x=-hw, y=-hl, z=0.0), Point(x=hw, y=-hl, z=0.0),
            Point(x=hw, y=hl, z=0.0)
        ]
        return m

    def crear_flecha(self, ns, id, pos, quat, color, stamp):
        m = Marker()
        m.header.frame_id = "world"
        m.header.stamp = stamp
        m.ns = ns
        m.id = id
        m.type = Marker.ARROW
        m.pose.position.x, m.pose.position.y, m.pose.position.z = pos
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = quat
        m.scale.x, m.scale.y, m.scale.z = 2.0, 0.2, 0.2
        m.color.r, m.color.g, m.color.b, m.color.a = color[0], color[1], color[2], 1.0
        return m

    def pose_to_numpy(self, pos): return np.array([pos.x, pos.y, pos.z])
    def quat_to_list(self, q): return [q.x, q.y, q.z, q.w]

def main(args=None):
    rclpy.init(args=args)
    nodo = RvizVisualizerNode()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            nodo.destroy_node()
            rclpy.shutdown()

if __name__ == "__main__":
    main()
