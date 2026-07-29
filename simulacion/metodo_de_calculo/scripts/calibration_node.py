#!/usr/bin/env python3

import json
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import String, Float64MultiArray
from std_srvs.srv import Trigger

class CalibrationNode(Node):
    """
    Node in charge of applying the mathematical HelioPoint method.
    Secure Industrial Version + Multi-Facet Support:
    - Alignment via pure Vectorial Cross Product.
    - Moving average for mirror defect cancellation.
    - Supports both monolithic collectors and facet meshes.
    """

    def __init__(self):
        super().__init__('calibration_node')
        
        self.theoretical_collectors = {} 
        self.error_history = {} 

        # --- CLIENTS AND SUBSCRIPTIONS ---
        self.cli_theory = self.create_client(Trigger, 'get_collector_theory')
        
        self.create_subscription(String, '/sim_status/collector_updates', self.update_theory_callback, 10)
        self.create_subscription(String, '/inspection/filtered_data', self.filtered_data_callback, 10)
        self.create_subscription(Float64MultiArray, '/control_param', self.param_callback, qos_profile_sensor_data)
        
        # --- PUBLISHERS ---
        self.pub_results = self.create_publisher(String, '/calibration/results', 10)
        self.error_pub = self.create_publisher(Float64MultiArray, '/heliostat_processed_errors', 10)

        self.focused_collector = None
        self.moving_average_buffer = [] 
        
        # Distance from the camera to the LED (in the camera's local system)
        self.d_cam_led = np.array([0.0, 0.0, -0.6])  
        
        # Default initial angle (45 degrees)
        self.cam_angle = 0.785  

        self.request_theoretical_map()
        self.get_logger().info("HelioPoint Brain started [UNIFIED FACET VERSION]. Waiting for theory...")

    def request_theoretical_map(self):
        if not self.cli_theory.service_is_ready():
            return
        req = Trigger.Request()
        self.cli_theory.call_async(req).add_done_callback(self.on_theory_received)

    # ==========================================
    # THE MAGIC: UNIFIED KINEMATICS IN MEMORY
    # ==========================================
    def on_theory_received(self, future):
        try:
            res = future.result()
            if res.success:
                c_list = json.loads(res.message)
                self.theoretical_collectors = {}
                
                for c in c_list:
                    # Base collector kinematics (Yaw -> Z, Pitch -> Y)
                    rot_yaw = R.from_euler('z', c.get('yaw', 0.0))
                    rot_pitch = R.from_euler('y', c.get('pitch', 0.0))
                    global_c_r = rot_yaw * rot_pitch
                    
                    global_c_pos = np.array([c.get('x', 0.0), c.get('y', 0.0), c.get('z', 0.0)])
                    
                    if 'facets' in c:
                        for f in c['facets']:
                            local_offset = np.array(f.get('offset', [0.0, 0.0, 0.0]))
                            pos_f = global_c_pos + global_c_r.apply(local_offset)
                            
                            # Local facet kinematics (Roll -> X, Pitch -> Y)
                            canting_rot = R.from_euler('xy', [f.get('cant_roll', 0.0), f.get('cant_pitch', 0.0)])
                            final_f_r = global_c_r * canting_rot
                            
                            # We store the pure Rotation object so no data is lost
                            self.theoretical_collectors[str(f['id'])] = {
                                'pos': pos_f,
                                'rot': final_f_r
                            }
                    else:
                        self.theoretical_collectors[str(c['id'])] = {
                            'pos': global_c_pos,
                            'rot': global_c_r
                        }
                        
                self.get_logger().info(f"Flattened Theoretical Plane: {len(self.theoretical_collectors)} trackable entities.")
        except Exception as e:
            self.get_logger().error(f"Error loading theory: {e}")

    def update_theory_callback(self, msg):
        self.request_theoretical_map()

    def param_callback(self, msg):
        if len(msg.data) >= 1: 
            self.cam_angle = float(msg.data[0])

    def calculate_measured_vector(self, p_cam, base_drone_r, p_reflex):
        # base_drone_r is the drone's rotation. We add the camera pitch.
        real_cam_r = base_drone_r * R.from_euler('y', self.cam_angle, degrees=False)
        p_led = p_cam + real_cam_r.apply(self.d_cam_led)
        
        reflected_v = p_cam - p_reflex
        unit_reflected_v = reflected_v / np.linalg.norm(reflected_v)
        
        incident_v = p_led - p_reflex
        unit_incident_v = incident_v / np.linalg.norm(incident_v)
        
        n_meas = unit_incident_v + unit_reflected_v
        return n_meas / np.linalg.norm(n_meas)

    def filtered_data_callback(self, msg):
        if not self.theoretical_collectors:
            self.get_logger().warn("Waiting for theoretical map...", throttle_duration_sec=3.0)
            return 

        try:
            filtered_data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error("Error: Invalid JSON message.", throttle_duration_sec=3.0)
            return

        if not filtered_data: return

        iteration_results = []

        for data in filtered_data:
            collector_id = str(data.get("collector_id"))
            
            # We look for the entity in memory
            theo_c = self.theoretical_collectors.get(collector_id)
            if not theo_c:
                continue
                
            p_theo = theo_c['pos']
            original_theo_r = theo_c['rot']
            
            # The theoretical normal of the mirror at rest (lying on XY) is the Z axis
            global_theo_n = original_theo_r.apply([0.0, 0.0, 1.0])
            
            try:
                p_cam = np.array(data["drone"]["pos"], dtype=float)
                drone_quat = np.array(data["drone"]["quat"], dtype=float)
                p_bounce_local = np.array(data["bounce_local"], dtype=float)
                
                if np.any(np.isnan(p_cam)) or np.any(np.isnan(drone_quat)) or np.any(np.isnan(p_bounce_local)):
                    continue
                    
                if np.linalg.norm(drone_quat) < 1e-6:
                    continue

                drone_r = R.from_quat(drone_quat)
                
            except (KeyError, ValueError, TypeError):
                continue
            
            r_iter = original_theo_r
            global_meas_n = np.array([0.0, 0.0, 0.0])
            
            # Newton-Raphson / Cross Product to find the real orientation
            for _ in range(3):
                global_reflex_p = p_theo + r_iter.apply(p_bounce_local)
                global_meas_n = self.calculate_measured_vector(p_cam, drone_r, global_reflex_p)
                global_curr_n = r_iter.apply([0.0, 0.0, 1.0])
                
                cross_prod = np.cross(global_curr_n, global_meas_n)
                norm_cross = np.linalg.norm(cross_prod)
                
                if norm_cross > 1e-8:
                    axis = cross_prod / norm_cross
                    dot_prod = np.dot(global_curr_n, global_meas_n)
                    dot_prod = max(-1.0, min(1.0, dot_prod))
                    angle = math.acos(dot_prod)
                    r_corr = R.from_rotvec(axis * angle)
                    r_iter = r_corr * r_iter 
            
            global_final_n = r_iter.apply([0.0, 0.0, 1.0])
            
            # We undo the global rotation to see the error in the mirror's local system
            ccs_final_n = original_theo_r.inv().apply(global_final_n)
            
            # We extract the error. 
            # X Rotation Error (Roll) and Y Rotation Error (Pitch)
            error_rotX_rad = -math.atan2(ccs_final_n[1], ccs_final_n[2])
            error_rotY_rad = math.atan2(ccs_final_n[0], ccs_final_n[2])
            
            rotX_mrad = error_rotX_rad * 1000.0
            rotY_mrad = error_rotY_rad * 1000.0
            
            MAX_SAMPLES = 50  
            
            if collector_id not in self.error_history:
                self.error_history[collector_id] = {"rotX": [], "rotY": []}
                
            self.error_history[collector_id]["rotX"].append(rotX_mrad)
            self.error_history[collector_id]["rotY"].append(rotY_mrad)
            
            if len(self.error_history[collector_id]["rotX"]) > MAX_SAMPLES:
                self.error_history[collector_id]["rotX"].pop(0)
                self.error_history[collector_id]["rotY"].pop(0)
                
            mean_rotX = float(np.mean(self.error_history[collector_id]["rotX"]))
            mean_rotY = float(np.mean(self.error_history[collector_id]["rotY"]))
            samples = len(self.error_history[collector_id]["rotX"])
            
            iteration_results.append({
                "id": collector_id, 
                "samples_taken": samples,
                "current_error_rotX_mrad": float(rotX_mrad),
                "current_error_rotY_mrad": float(rotY_mrad),
                "error_x_mrad": mean_rotX, 
                "error_y_mrad": mean_rotY,
                "theoretical_normal": global_theo_n.tolist(),
                "measured_normal": global_meas_n.tolist(),
                "bounce_local": p_bounce_local.tolist()
            })

        if iteration_results:
            msg_pub = String(data=json.dumps(iteration_results))
            self.pub_results.publish(msg_pub)

def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt: 
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
