#!/usr/bin/env python3

import json
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point, PoseArray, PoseStamped, Pose
from scipy.spatial.transform import Rotation as R
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Trigger
from rclpy.qos import qos_profile_sensor_data

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
        self.paneles_reales = []
        self.impactos_data = [] 

        # --- CLIENTE DE SERVICIO ---
        self.cli_realidad = self.create_client(Trigger, 'get_panel_real')

        # --- PUBLICADOR ---
        self.pub_marcadores = self.create_publisher(MarkerArray, '/visualization/scene', 10)
        
        # --- SUSCRIPCIONES ---
        # --- SUSCRIPCIONES ---
        self.create_subscription(String, '/sim_status/panel_updates', self.actualizar_paneles_callback, 10)
        self.create_subscription(PoseStamped, '/data/drone', self.dron_callback, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/data/camera', self.camara_callback, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/data/light', self.luz_callback, qos_profile_sensor_data)
        self.create_subscription(String, '/inspection/raw_data', self.raw_data_callback, qos_profile_sensor_data)
        

        # --- TIMER DE DIBUJO (10 Hz) ---
        self.timer = self.create_timer(0.1, self.publicar_escena)

        self.pedir_mapa_inicial()
        self.get_logger().info("Visualizador RViz iniciado. Modo Facetas activado.")

    # ==========================================
    # OBTENCIÓN DE MAPA (Igual que la VirtualCam)
    # ==========================================
    def pedir_mapa_inicial(self):
        if not self.cli_realidad.service_is_ready(): return
        req = Trigger.Request()
        self.cli_realidad.call_async(req).add_done_callback(self.al_recibir_mapa_srv)

    def al_recibir_mapa_srv(self, futuro):
        try:
            res = futuro.result()
            self.paneles_reales = json.loads(res.message)
        except Exception as e:
            self.get_logger().error(f"Error cargando mapa para RViz: {e}")

    def actualizar_paneles_callback(self, msg):
        self.pedir_mapa_inicial()

    # ==========================================
    # CALLBACKS
    # ==========================================

    def dron_callback(self, msg): 
        self.dron_pose = msg

    def camara_callback(self, msg): 
        self.cam_pose = msg

    def luz_callback(self, msg): 
        self.luz_pose = msg
    
    def raw_data_callback(self, msg):
        try:
            self.impactos_data = json.loads(msg.data)
        except:
            self.impactos_data = []

    # ==========================================
    # BUCLE DE RENDERIZADO 3D (RVIZ)
    # ==========================================

    def publicar_escena(self):
        if not self.dron_pose:
            return

        msg_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        # 1. DIBUJAR PANELES (Soporte para MODO AVANZADO)
        marker_id = 0
        for p in self.paneles_reales:
            pos_p_global = np.array([p['x'], p['y'], p['z']])
            r_p_global = R.from_euler('xyz', [0.0, p['pitch'], p['yaw']])

            if 'facetas' in p:
                w_f = p.get('width_x', 10.4) / 5.0
                l_f = p.get('length_y', 11.4) / 5.0
                for f in p['facetas']:
                    pos_f = pos_p_global + r_p_global.apply(f['offset'])
                    r_f = r_p_global * R.from_euler('xyz', [0.0, f['cant_pitch'], f['cant_yaw']])
                    
                    pose = Pose()
                    pose.position.x, pose.position.y, pose.position.z = pos_f
                    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = r_f.as_quat()
                    
                    # Le pasamos el ancho/largo exacto de la faceta
                    msg_array.markers.append(self.crear_marcador_panel(marker_id, pose, w_f/2, l_f/2, stamp))
                    marker_id += 1
            else:
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = pos_p_global
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = r_p_global.as_quat()
                
                w = p.get('width_x', 10.4) / 2
                l = p.get('length_y', 11.4) / 2
                msg_array.markers.append(self.crear_marcador_panel(marker_id, pose, w, l, stamp))
                marker_id += 1

        # 2. DIBUJAR DRON Y CÁMARA
        p_dron = self.pose_to_numpy(self.dron_pose.pose.position)
        q_dron = self.quat_to_list(self.dron_pose.pose.orientation)
        msg_array.markers.append(self.crear_flecha("dron", 998, p_dron, q_dron, [0.0, 0.5, 1.0], stamp))

        if self.cam_pose:
            q_cam = self.quat_to_list(self.cam_pose.pose.orientation)
            msg_array.markers.append(self.crear_flecha("camara", 999, p_dron, q_cam, [1.0, 0.0, 0.0], stamp))

        # 3. DIBUJAR RAYOS / IMPACTOS
        if self.luz_pose:
            for i, impacto in enumerate(self.impactos_data):
                p_rebote_world = np.array(impacto.get("rebote_world_debug", [0,0,0]))
                if not np.array_equal(p_rebote_world, [0,0,0]):
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

    def crear_marcador_panel(self, id, pose, hw, hl, stamp):
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
        
        # Ahora recibe hw (Half-Width) y hl (Half-Length) dinámicamente
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
