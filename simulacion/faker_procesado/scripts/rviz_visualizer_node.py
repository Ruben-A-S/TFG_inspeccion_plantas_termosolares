#!/usr/bin/env python3

import json
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Trigger
from builtin_interfaces.msg import Time, Duration
from scipy.spatial.transform import Rotation as R # Añadido para el canting estático

class RvizVisualizerNode(Node):
    def __init__(self):
        super().__init__('rviz_visualizer_node')
        
        self.dron_pose = None
        self.cam_pose = None
        self.luz_pose = None
        self.impactos_data = [] 

        self.cli_realidad = self.create_client(Trigger, 'get_panel_real')
        self.pub_marcadores = self.create_publisher(MarkerArray, '/visualization/scene', 10)
        
        self.create_subscription(String, '/sim_status/panel_updates', self.actualizar_paneles_callback, 10)
        self.create_subscription(PoseStamped, '/data/drone', self.dron_callback, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/data/camera', self.camara_callback, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/data/light', self.luz_callback, qos_profile_sensor_data)
        self.create_subscription(String, '/inspection/raw_data', self.raw_data_callback, qos_profile_sensor_data)
        
        self.timer = self.create_timer(0.1, self.publicar_escena_dinamica)
        self.pedir_mapa_inicial()
        self.get_logger().info("Visualizador RViz [Zero-Math Dinámico + Anclaje Base] Inicializado.")

    def pedir_mapa_inicial(self):
        if not self.cli_realidad.service_is_ready(): return
        req = Trigger.Request()
        self.cli_realidad.call_async(req).add_done_callback(self.al_recibir_mapa_srv)

    def al_recibir_mapa_srv(self, futuro):
        try:
            res = futuro.result()
            paneles = json.loads(res.message)
            msg_array = MarkerArray()
            stamp_zero = Time(sec=0, nanosec=0)
            
            m_clear = Marker()
            m_clear.action = Marker.DELETEALL
            msg_array.markers.append(m_clear)
            
            marker_id = 0
            for p in paneles:
                id_panel = p['id']
                
                if 'facetas' in p:
                    w_f = p.get('width_x', 10.4) / 5.0
                    l_f = p.get('length_y', 11.4) / 5.0
                    for f in p['facetas']:
                        m = Marker()
                        # TRUCO: Anclamos a la inclinacion global, que SÍ se publica en TF2
                        m.header.frame_id = f"inclinacion_{id_panel}" 
                        m.header.stamp = stamp_zero               
                        m.ns = "heliostatos_estaticos"
                        m.id = marker_id
                        m.type = Marker.LINE_STRIP
                        m.action = Marker.ADD
                        
                        # Le aplicamos la pose local (solo se calcula una vez al cargar)
                        offset = f.get('offset', [0.0, 0.0, 0.0])
                        m.pose.position.x = float(offset[0])
                        m.pose.position.y = float(offset[1])
                        m.pose.position.z = float(offset[2])
                        
                        rot_canting = R.from_euler('xy', [f.get('cant_roll', 0.0), f.get('cant_pitch', 0.0)])
                        q = rot_canting.as_quat()
                        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = q[0], q[1], q[2], q[3]
                        
                        m.scale.x = 0.1 
                        m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 1.0
                        
                        hw, hl = w_f / 2.0, l_f / 2.0
                        m.points = [
                            Point(x=hw, y=hl, z=0.0), Point(x=-hw, y=hl, z=0.0),
                            Point(x=-hw, y=-hl, z=0.0), Point(x=hw, y=-hl, z=0.0),
                            Point(x=hw, y=hl, z=0.0)
                        ]
                        msg_array.markers.append(m)
                        marker_id += 1
                else:
                    m = Marker()
                    m.header.frame_id = f"inclinacion_{id_panel}"
                    m.header.stamp = stamp_zero
                    m.ns = "heliostatos_estaticos"
                    m.id = marker_id
                    m.type = Marker.LINE_STRIP
                    m.action = Marker.ADD
                    m.pose.orientation.w = 1.0
                    m.scale.x = 0.1
                    m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 1.0
                    
                    hw = p.get('width_x', 10.4) / 2.0
                    hl = p.get('length_y', 11.4) / 2.0
                    m.points = [
                        Point(x=hw, y=hl, z=0.0), Point(x=-hw, y=hl, z=0.0),
                        Point(x=-hw, y=-hl, z=0.0), Point(x=hw, y=-hl, z=0.0),
                        Point(x=hw, y=hl, z=0.0)
                    ]
                    msg_array.markers.append(m)
                    marker_id += 1
                    
            self.pub_marcadores.publish(msg_array)
        except Exception as e:
            self.get_logger().error(f"Error cargando mapa en RViz: {e}")

    def actualizar_paneles_callback(self, msg):
        self.pedir_mapa_inicial()

    def dron_callback(self, msg): self.dron_pose = msg
    def camara_callback(self, msg): self.cam_pose = msg
    def luz_callback(self, msg): self.luz_pose = msg
    def raw_data_callback(self, msg):
        try: self.impactos_data = json.loads(msg.data)
        except: self.impactos_data = []

    def publicar_escena_dinamica(self):
        if not self.dron_pose: return

        msg_array = MarkerArray()
        stamp_ahora = self.get_clock().now().to_msg()
        lifetime_msg = Duration(sec=0, nanosec=150000000)

        m_dron = Marker()
        m_dron.header.frame_id, m_dron.header.stamp = "world", stamp_ahora
        m_dron.ns, m_dron.id, m_dron.type = "dron", 998, Marker.ARROW
        m_dron.pose = self.dron_pose.pose
        m_dron.scale.x, m_dron.scale.y, m_dron.scale.z = 2.0, 0.2, 0.2
        m_dron.color.r, m_dron.color.g, m_dron.color.b, m_dron.color.a = 0.0, 0.5, 1.0, 1.0
        m_dron.lifetime = lifetime_msg
        msg_array.markers.append(m_dron)

        if self.cam_pose:
            m_cam = Marker()
            m_cam.header.frame_id, m_cam.header.stamp = "world", stamp_ahora
            m_cam.ns, m_cam.id, m_cam.type = "camara", 999, Marker.ARROW
            m_cam.pose = self.cam_pose.pose
            m_cam.scale.x, m_cam.scale.y, m_cam.scale.z = 2.0, 0.2, 0.2
            m_cam.color.r, m_cam.color.g, m_cam.color.b, m_cam.color.a = 1.0, 0.0, 0.0, 1.0
            m_cam.lifetime = lifetime_msg
            msg_array.markers.append(m_cam)

        if self.luz_pose:
            for i, impacto in enumerate(self.impactos_data):
                p_rebote_world = impacto.get("rebote_world_debug", [0, 0, 0])
                if p_rebote_world != [0, 0, 0]:
                    m_imp = Marker()
                    m_imp.header.frame_id, m_imp.header.stamp = "world", stamp_ahora
                    m_imp.ns, m_imp.id, m_imp.type = "impactos", i, Marker.SPHERE
                    m_imp.pose.position.x, m_imp.pose.position.y, m_imp.pose.position.z = p_rebote_world
                    m_imp.scale.x, m_imp.scale.y, m_imp.scale.z = 0.4, 0.4, 0.4
                    m_imp.color.r, m_imp.color.g, m_imp.color.b, m_imp.color.a = 1.0, 1.0, 0.0, 1.0
                    m_imp.lifetime = lifetime_msg
                    msg_array.markers.append(m_imp)

        self.pub_marcadores.publish(msg_array)


def main(args=None):
    rclpy.init(args=args)
    nodo = RvizVisualizerNode()
    try: rclpy.spin(nodo)
    except KeyboardInterrupt: pass
    finally:
        if rclpy.ok():
            nodo.destroy_node()
            rclpy.shutdown()

if __name__ == "__main__":
    main()
