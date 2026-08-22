#!/usr/bin/env python3

import json
import subprocess
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation as R
from scipy.spatial import KDTree
from std_msgs.msg import Float64MultiArray, String
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from std_srvs.srv import Trigger

class OpticsCalculatorNode(Node):
    def __init__(self, world_name="test1", drone_model="x500"):
        super().__init__('optics_calculator_node')
        
        # --- 1. DECLARAR PARÁMETROS DEL YAML ---
        self.declare_parameter('ui_defaults.world_name', 'test1')
        self.declare_parameter('ui_defaults.drone_model', 'x500')
        self.declare_parameter('ui_defaults.camera_pitch_deg', 45.0)
        
        self.declare_parameter('camera.optics_fov_degrees', 90.0)
        self.declare_parameter('camera.max_vision_dist', 500.0)
        self.declare_parameter('camera.led_offset', [0.0, 0.0, -0.6])
        
        self.declare_parameter('collector.default_width', 10.4)
        self.declare_parameter('collector.default_length', 11.4)
        self.declare_parameter('collector.facet_cols', 5)
        self.declare_parameter('collector.facet_rows', 5)
        
        self.declare_parameter('performance.optics_hz', 20.0)

        # --- 2. EXTRAER VALORES ---
        self.world_name = self.get_parameter('ui_defaults.world_name').value
        self.drone_model = self.get_parameter('ui_defaults.drone_model').value
        
        # Transformamos directamente a radianes al leer
        pitch_deg = self.get_parameter('ui_defaults.camera_pitch_deg').value
        self.cam_angle = np.radians(pitch_deg)
        
        self.fov_degrees = self.get_parameter('camera.optics_fov_degrees').value
        self.fov_cosine_limit = np.cos(np.radians(self.fov_degrees / 2.0))
        self.max_vision_distance = self.get_parameter('camera.max_vision_dist').value
        self.led_offset = self.get_parameter('camera.led_offset').value
        
        self.def_w = self.get_parameter('collector.default_width').value
        self.def_l = self.get_parameter('collector.default_length').value
        self.f_cols = self.get_parameter('collector.facet_cols').value
        self.f_rows = self.get_parameter('collector.facet_rows').value
        
        optics_hz = self.get_parameter('performance.optics_hz').value

        # PURE RAM MEMORY: We store the JSON here
        self.drone_pos = None
        self.drone_quat = None
        self.new_pose_available = False
        
        self.memory_collectors = []
        self.collector_coords = []
        self.collector_sizes = {}
        self.kd_tree = None

        self.cli_real = self.create_client(Trigger, 'get_collector_real')
        
        self.pub_raw_data = self.create_publisher(String, '/inspection/raw_data', qos_profile_sensor_data)
        self.pub_bounces_viz = self.create_publisher(PoseArray, '/data/impacts', qos_profile_sensor_data)
        self.pub_drone = self.create_publisher(PoseStamped, '/data/drone', qos_profile_sensor_data)
        self.pub_camera = self.create_publisher(PoseStamped, '/data/camera', qos_profile_sensor_data)
        self.pub_light = self.create_publisher(PoseStamped, '/data/light', qos_profile_sensor_data)

        self.create_subscription(Float64MultiArray, '/control_param', self.param_callback, qos_profile_sensor_data)
        self.create_subscription(String, '/sim_status/collector_updates', self.collector_update_callback, 10)

        timer_period = 1.0 / optics_hz if optics_hz > 0 else 0.05
        self.create_timer(timer_period, self.perception_loop_hz)

        self.request_full_map()
        self.launch_gazebo_spy()
        self.get_logger().info("Optics Calculator [KD-Tree + Pure RAM] started. Zero latency.")

    def collector_update_callback(self, msg):
        self.request_full_map()

    def request_full_map(self):
        if not self.cli_real.service_is_ready(): return
        req = Trigger.Request()
        self.cli_real.call_async(req).add_done_callback(self.on_map_received)

    def on_map_received(self, future):
        try:
            res = future.result()
            if res.success:
                self.memory_collectors = json.loads(res.message)
                self.collector_names = [c['id'] for c in self.memory_collectors]
                self.collector_coords = [[c['x'], c['y'], c['z']] for c in self.memory_collectors]
                
                if self.collector_coords:
                    self.kd_tree = KDTree(self.collector_coords)
                
                self.collector_sizes = {
                    c['id']: {'w': c.get('width_x', self.def_w), 'l': c.get('length_y', self.def_l)} 
                    for c in self.memory_collectors
                }
        except Exception as e:
            self.get_logger().error(f"Error processing metadata: {e}")

    def param_callback(self, msg):
        if len(msg.data) >= 1: self.cam_angle = msg.data[0]

    def perception_loop_hz(self):
        if not self.new_pose_available or self.drone_pos is None or self.kd_tree is None:
            return
            
        self.new_pose_available = False 
        stamp = self.get_clock().now().to_msg()
        
        rot_drone = R.from_quat(self.drone_quat)
        rot_cam = rot_drone * R.from_euler('y', self.cam_angle)
        optical_axis = rot_cam.apply([1, 0, 0])
        
        msg_drone = PoseStamped()
        msg_drone.header.frame_id, msg_drone.header.stamp = "world", stamp
        msg_drone.pose.position.x, msg_drone.pose.position.y, msg_drone.pose.position.z = self.drone_pos
        msg_drone.pose.orientation.x, msg_drone.pose.orientation.y, msg_drone.pose.orientation.z, msg_drone.pose.orientation.w = self.drone_quat
        self.pub_drone.publish(msg_drone)

        msg_cam = PoseStamped()
        msg_cam.header.frame_id, msg_cam.header.stamp = "world", stamp
        msg_cam.pose.position = msg_drone.pose.position
        msg_cam.pose.orientation.x, msg_cam.pose.orientation.y, msg_cam.pose.orientation.z, msg_cam.pose.orientation.w = rot_cam.as_quat()
        self.pub_camera.publish(msg_cam)

        light_pos = self.drone_pos + rot_cam.apply(self.led_offset)
        msg_light = PoseStamped()
        msg_light.header.frame_id, msg_light.header.stamp = "world", stamp
        msg_light.pose.position.x, msg_light.pose.position.y, msg_light.pose.position.z = light_pos
        self.pub_light.publish(msg_light)

        impacts = []
        msg_viz = PoseArray()
        msg_viz.header.frame_id, msg_viz.header.stamp = "world", stamp

        nearby_indices = self.kd_tree.query_ball_point(self.drone_pos, r=self.max_vision_distance)

        for idx in nearby_indices:
            c = self.memory_collectors[idx]
            collector_id = c['id']
            global_c_pos = np.array([c['x'], c['y'], c['z']])
            
            pole_dir_vec = global_c_pos - self.drone_pos
            dist = np.linalg.norm(pole_dir_vec)
            if dist < 1e-3: continue
            
            angle_cosine = np.dot(optical_axis, pole_dir_vec / dist)
            if angle_cosine < self.fov_cosine_limit:
                continue 
                
            size = self.collector_sizes.get(collector_id, {'w': self.def_w, 'l': self.def_l})
            w_f = size['w'] / float(self.f_cols)
            l_f = size['l'] / float(self.f_rows)

            # YOUR ORIGINAL KINEMATICS (Synchronous and robust)
            rot_yaw = R.from_euler('z', c.get('yaw', 0.0))
            rot_pitch = R.from_euler('y', c.get('pitch', 0.0))
            global_c_r = rot_yaw * rot_pitch

            if 'facets' in c:
                for f in c['facets']:
                    raw_facet_id = f"{collector_id}_f{f['id']}" if not f['id'].startswith(collector_id) else f['id']
                    
                    local_offset = np.array(f.get('offset', [0.0, 0.0, 0.0]))
                    pos_f = global_c_pos + global_c_r.apply(local_offset)
                    
                    canting_rot = R.from_euler('xy', [f.get('cant_roll', 0.0), f.get('cant_pitch', 0.0)])
                    rot_f = global_c_r * canting_rot

                    inv_rot_f = rot_f.inv()
                    cam_loc = inv_rot_f.apply(self.drone_pos - pos_f)
                    light_loc = inv_rot_f.apply(light_pos - pos_f)

                    if cam_loc[2] <= 0: continue 

                    ref_loc = np.array([light_loc[0], light_loc[1], -light_loc[2]])
                    denom = cam_loc[2] - ref_loc[2]
                    if abs(denom) < 1e-6: continue
                    
                    i_loc = ref_loc + (-ref_loc[2] / denom) * (cam_loc - ref_loc)

                    if abs(i_loc[0]) <= (w_f/2) and abs(i_loc[1]) <= (l_f/2):
                        i_world = pos_f + rot_f.apply(i_loc)
                        impacts.append({
                            "collector_id": raw_facet_id,
                            "bounce_local": i_loc.tolist(),
                            "bounce_world_debug": i_world.tolist(),
                            "drone": {
                                "pos": self.drone_pos.tolist(),
                                "quat": self.drone_quat.tolist()
                            }
                        })
                        pv = Pose()
                        pv.position.x, pv.position.y, pv.position.z = i_world
                        msg_viz.poses.append(pv)
            else:
                pass # Simple collector logic if applicable

        if impacts:
            self.pub_raw_data.publish(String(data=json.dumps(impacts)))
            self.pub_bounces_viz.publish(msg_viz)

    def launch_gazebo_spy(self):
        self.gz_thread = threading.Thread(target=self.listen_gazebo, daemon=True)
        self.gz_thread.start()

    def listen_gazebo(self):
        # LINUX FILTER: Extracts only the drone from the global topic
        command = f"gz topic -e -t /world/{self.world_name}/pose/info | grep --line-buffered -A 12 'name: \"{self.drone_model}_0\"'"
        proc = subprocess.Popen(command, stdout=subprocess.PIPE, text=True, shell=True)
        
        in_pos, in_ori = False, False
        c_p = [0.0, 0.0, 0.0]
        c_q = [0.0, 0.0, 0.0, 1.0]

        for line in iter(proc.stdout.readline, ''):
            line = line.strip()
            if "position" in line: in_pos, in_ori = True, False; continue
            if "orientation" in line: in_pos, in_ori = False, True; continue
            
            if in_pos:
                if "x:" in line: c_p[0] = float(line.split(":")[1])
                if "y:" in line: c_p[1] = float(line.split(":")[1])
                if "z:" in line: c_p[2] = float(line.split(":")[1])
            if in_ori:
                if "x:" in line: c_q[0] = float(line.split(":")[1])
                if "y:" in line: c_q[1] = float(line.split(":")[1])
                if "z:" in line: c_q[2] = float(line.split(":")[1])
                if "w:" in line: c_q[3] = float(line.split(":")[1])

            if "}" in line and in_ori:
                self.drone_pos, self.drone_quat = np.array(c_p), np.array(c_q)
                self.new_pose_available = True  
                in_pos = in_ori = False

def main(args=None):
    rclpy.init(args=args)
    node = OpticsCalculatorNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        if rclpy.ok(): node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__": main()
