#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
import json
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from scipy.spatial import KDTree
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rclpy.qos import qos_profile_sensor_data

class VirtualCameraNode(Node):
    def __init__(self):
        super().__init__('virtual_camera_node')
        
        self.br = CvBridge()
        self.res_w, self.res_h = 640, 480
        self.focal_dist = 1.5
        self.sensor_w, self.sensor_h = 1.6, 1.2
        
        self.fov_cosine_limit = np.cos(np.radians(160.0 / 2.0))
        self.draw_distance = 150.0
        
        self.drone_pose = None
        self.cam_pose = None
        self.recent_impacts = []
        
        self.memory_collectors = []
        self.collector_coords = []
        self.kd_tree = None

        self.cli_real = self.create_client(Trigger, 'get_collector_real')

        self.pub_image = self.create_publisher(Image, '/virtual_camera/image', 10)
        self.create_subscription(String, '/sim_status/collector_updates', self.update_collectors_callback, 10)
        self.create_subscription(PoseStamped, '/data/drone', self.drone_callback, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/data/camera', self.camera_callback, qos_profile_sensor_data)
        self.create_subscription(String, '/inspection/raw_data', self.raw_data_callback, qos_profile_sensor_data)
        
        self.create_timer(0.05, self.render_loop)
        self.request_initial_map()
        self.get_logger().info("Virtual Camera [KD-Tree + Pure RAM] active.")

    def request_initial_map(self):
        if not self.cli_real.service_is_ready(): return
        req = Trigger.Request()
        self.cli_real.call_async(req).add_done_callback(self.on_map_received_srv)

    def on_map_received_srv(self, future):
        try:
            res = future.result()
            self.memory_collectors = json.loads(res.message)
            self.collector_coords = [[c['x'], c['y'], c['z']] for c in self.memory_collectors]
            if self.collector_coords:
                self.kd_tree = KDTree(self.collector_coords)
        except Exception as e: self.get_logger().error(f"Map error: {e}")

    def update_collectors_callback(self, msg):
        self.request_initial_map()

    def drone_callback(self, msg): self.drone_pose = msg
    def camera_callback(self, msg): self.cam_pose = msg
    def raw_data_callback(self, msg):
        try: self.recent_impacts = json.loads(msg.data)
        except: self.recent_impacts = []

    def render_loop(self):
        img = np.zeros((self.res_h, self.res_w, 3), dtype=np.uint8)
        if not self.cam_pose or not self.kd_tree:
            self.pub_image.publish(self.br.cv2_to_imgmsg(img, encoding="bgr8"))
            return

        p_cam = np.array([self.cam_pose.pose.position.x, self.cam_pose.pose.position.y, self.cam_pose.pose.position.z])
        q_cam = self.cam_pose.pose.orientation
        r_cam_inv = R.from_quat([q_cam.x, q_cam.y, q_cam.z, q_cam.w]).inv()
        optical_axis = R.from_quat([q_cam.x, q_cam.y, q_cam.z, q_cam.w]).apply([1, 0, 0])

        nearby_indices = self.kd_tree.query_ball_point(p_cam, r=self.draw_distance)

        for idx in nearby_indices:
            c = self.memory_collectors[idx] 
            global_c_pos = np.array([c['x'], c['y'], c['z']])
            
            dir_vec = global_c_pos - p_cam
            dist = np.linalg.norm(dir_vec)
            if dist < 1e-3: continue
            if np.dot(optical_axis, dir_vec/dist) < self.fov_cosine_limit: continue 

            # YOUR ORIGINAL KINEMATICS (Synchronous)
            rot_yaw = R.from_euler('z', c.get('yaw', 0.0))
            rot_pitch = R.from_euler('y', c.get('pitch', 0.0))
            global_c_r = rot_yaw * rot_pitch

            draw_targets = []
            if 'facets' in c:
                w_f, l_f = c.get('width_x', 10.4) / 5.0, c.get('length_y', 11.4) / 5.0
                for f in c['facets']:
                    local_offset = np.array(f.get('offset', [0.0, 0.0, 0.0]))
                    global_f_pos = global_c_pos + global_c_r.apply(local_offset)
                    
                    canting_rot = R.from_euler('xy', [f.get('cant_roll', 0.0), f.get('cant_pitch', 0.0)])
                    final_f_r = global_c_r * canting_rot
                    
                    draw_targets.append({'pos': global_f_pos, 'rot_mat': final_f_r, 'w': w_f, 'l': l_f})
            else:
                draw_targets.append({'pos': global_c_pos, 'rot_mat': global_c_r, 'w': c.get('width_x', 10.4), 'l': c.get('length_y', 11.4)})

            for obj in draw_targets:
                hw, hl = obj['w']/2, obj['l']/2
                local_corners = np.array([[hw, hl, 0], [-hw, hl, 0], [-hw, -hl, 0], [hw, -hl, 0]])
                world_corners = obj['rot_mat'].apply(local_corners) + obj['pos']
                cam_corners = r_cam_inv.apply(world_corners - p_cam)
                
                # Vectorized Anti-Flicker Projection
                if np.all(cam_corners[:, 0] > 0.1):
                    u = (((-self.focal_dist * (cam_corners[:, 1] / cam_corners[:, 0])) / self.sensor_w) + 0.5) * self.res_w
                    v = (((-self.focal_dist * (cam_corners[:, 2] / cam_corners[:, 0])) / self.sensor_h) + 0.5) * self.res_h
                    
                    pts = np.vstack((u, v)).T.astype(np.int32)
                    cv2.polylines(img, [pts], isClosed=True, color=(0, 255, 0), thickness=2)

        if self.recent_impacts:
            world_pts = np.array([imp.get('bounce_world_debug', [0,0,0]) for imp in self.recent_impacts])
            cam_pts = r_cam_inv.apply(world_pts - p_cam)
            front_mask = cam_pts[:, 0] > 0.1
            if np.any(front_mask):
                valid_p = cam_pts[front_mask]
                u = (((-self.focal_dist * (valid_p[:, 1] / valid_p[:, 0])) / self.sensor_w) + 0.5) * self.res_w
                v = (((-self.focal_dist * (valid_p[:, 2] / valid_p[:, 0])) / self.sensor_h) + 0.5) * self.res_h
                inside_mask = (u >= 0) & (u < self.res_w) & (v >= 0) & (v < self.res_h)
                for ui, vi in zip(u[inside_mask], v[inside_mask]):
                    cv2.circle(img, (int(ui), int(vi)), 8, (0, 255, 255), -1)

        self.pub_image.publish(self.br.cv2_to_imgmsg(img, encoding="bgr8"))

def main(args=None):
    rclpy.init(args=args)
    node = VirtualCameraNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__": main()
