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
from scipy.spatial.transform import Rotation as R # Added for static canting

class RvizVisualizerNode(Node):
    def __init__(self):
        super().__init__('rviz_visualizer_node')
        
        self.drone_pose = None
        self.cam_pose = None
        self.light_pose = None
        self.impacts_data = [] 

        self.cli_real = self.create_client(Trigger, 'get_collector_real')
        self.pub_markers = self.create_publisher(MarkerArray, '/visualization/scene', 10)
        
        self.create_subscription(String, '/sim_status/collector_updates', self.update_collectors_callback, 10)
        self.create_subscription(PoseStamped, '/data/drone', self.drone_callback, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/data/camera', self.camera_callback, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/data/light', self.light_callback, qos_profile_sensor_data)
        self.create_subscription(String, '/inspection/raw_data', self.raw_data_callback, qos_profile_sensor_data)
        
        self.timer = self.create_timer(0.1, self.publish_dynamic_scene)
        self.request_initial_map()
        self.get_logger().info("RViz Visualizer [Dynamic Zero-Math + Base Anchor] Initialized.")

    def request_initial_map(self):
        if not self.cli_real.service_is_ready(): return
        req = Trigger.Request()
        self.cli_real.call_async(req).add_done_callback(self.on_map_received_srv)

    def on_map_received_srv(self, future):
        try:
            res = future.result()
            collectors = json.loads(res.message)
            msg_array = MarkerArray()
            stamp_zero = Time(sec=0, nanosec=0)
            
            m_clear = Marker()
            m_clear.action = Marker.DELETEALL
            msg_array.markers.append(m_clear)
            
            marker_id = 0
            for c in collectors:
                collector_id = c['id']
                
                if 'facets' in c:
                    w_f = c.get('width_x', 10.4) / 5.0
                    l_f = c.get('length_y', 11.4) / 5.0
                    for f in c['facets']:
                        m = Marker()
                        # TRICK: We anchor to the global inclination, which IS published in TF2
                        m.header.frame_id = f"inclination_{collector_id}" 
                        m.header.stamp = stamp_zero               
                        m.ns = "static_heliostats"
                        m.id = marker_id
                        m.type = Marker.LINE_STRIP
                        m.action = Marker.ADD
                        
                        # We apply the local pose (only calculated once upon loading)
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
                    m.header.frame_id = f"inclination_{collector_id}"
                    m.header.stamp = stamp_zero
                    m.ns = "static_heliostats"
                    m.id = marker_id
                    m.type = Marker.LINE_STRIP
                    m.action = Marker.ADD
                    m.pose.orientation.w = 1.0
                    m.scale.x = 0.1
                    m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 1.0
                    
                    hw = c.get('width_x', 10.4) / 2.0
                    hl = c.get('length_y', 11.4) / 2.0
                    m.points = [
                        Point(x=hw, y=hl, z=0.0), Point(x=-hw, y=hl, z=0.0),
                        Point(x=-hw, y=-hl, z=0.0), Point(x=hw, y=-hl, z=0.0),
                        Point(x=hw, y=hl, z=0.0)
                    ]
                    msg_array.markers.append(m)
                    marker_id += 1
                    
            self.pub_markers.publish(msg_array)
        except Exception as e:
            self.get_logger().error(f"Error loading map in RViz: {e}")

    def update_collectors_callback(self, msg):
        self.request_initial_map()

    def drone_callback(self, msg): self.drone_pose = msg
    def camera_callback(self, msg): self.cam_pose = msg
    def light_callback(self, msg): self.light_pose = msg
    def raw_data_callback(self, msg):
        try: self.impacts_data = json.loads(msg.data)
        except: self.impacts_data = []

    def publish_dynamic_scene(self):
        if not self.drone_pose: return

        msg_array = MarkerArray()
        now_stamp = self.get_clock().now().to_msg()
        lifetime_msg = Duration(sec=0, nanosec=150000000)

        m_drone = Marker()
        m_drone.header.frame_id, m_drone.header.stamp = "world", now_stamp
        m_drone.ns, m_drone.id, m_drone.type = "drone", 998, Marker.ARROW
        m_drone.pose = self.drone_pose.pose
        m_drone.scale.x, m_drone.scale.y, m_drone.scale.z = 2.0, 0.2, 0.2
        m_drone.color.r, m_drone.color.g, m_drone.color.b, m_drone.color.a = 0.0, 0.5, 1.0, 1.0
        m_drone.lifetime = lifetime_msg
        msg_array.markers.append(m_drone)

        if self.cam_pose:
            m_cam = Marker()
            m_cam.header.frame_id, m_cam.header.stamp = "world", now_stamp
            m_cam.ns, m_cam.id, m_cam.type = "camera", 999, Marker.ARROW
            m_cam.pose = self.cam_pose.pose
            m_cam.scale.x, m_cam.scale.y, m_cam.scale.z = 2.0, 0.2, 0.2
            m_cam.color.r, m_cam.color.g, m_cam.color.b, m_cam.color.a = 1.0, 0.0, 0.0, 1.0
            m_cam.lifetime = lifetime_msg
            msg_array.markers.append(m_cam)

        if self.light_pose:
            for i, impact in enumerate(self.impacts_data):
                p_bounce_world = impact.get("bounce_world_debug", [0, 0, 0])
                if p_bounce_world != [0, 0, 0]:
                    m_imp = Marker()
                    m_imp.header.frame_id, m_imp.header.stamp = "world", now_stamp
                    m_imp.ns, m_imp.id, m_imp.type = "impacts", i, Marker.SPHERE
                    m_imp.pose.position.x, m_imp.pose.position.y, m_imp.pose.position.z = p_bounce_world
                    m_imp.scale.x, m_imp.scale.y, m_imp.scale.z = 0.4, 0.4, 0.4
                    m_imp.color.r, m_imp.color.g, m_imp.color.b, m_imp.color.a = 1.0, 1.0, 0.0, 1.0
                    m_imp.lifetime = lifetime_msg
                    msg_array.markers.append(m_imp)

        self.pub_markers.publish(msg_array)


def main(args=None):
    rclpy.init(args=args)
    node = RvizVisualizerNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == "__main__":
    main()
