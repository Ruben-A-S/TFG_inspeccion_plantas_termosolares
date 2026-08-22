#!/usr/bin/env python3

import json
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Time, Duration
from geometry_msgs.msg import Point
import numpy as np
from scipy.spatial.transform import Rotation as R

class RVizCalibrationMarkersNode(Node):
    """
    Node in charge of subscribing to HelioPoint results
    and publishing MarkerArrays in RViz to visualize normal vectors,
    including the reconstruction of the Cumulative Mean Normal.
    Adapted to support Multi-facet Heliostats.
    """
    def __init__(self):
        super().__init__('rviz_calibration_markers_node')
        
        # --- 1. DECLARAR PARÁMETROS DEL YAML ---
        self.declare_parameter('visualization.calibration_arrow_lifetime_s', 2.0)
        self.declare_parameter('visualization.rviz_arrow_length', 3.0)
        self.declare_parameter('visualization.rviz_arrow_thickness', 0.1)
        
        # --- 2. EXTRAER VALORES ---
        lifetime_s = self.get_parameter('visualization.calibration_arrow_lifetime_s').value
        self.arrow_length = self.get_parameter('visualization.rviz_arrow_length').value
        self.arrow_thick = self.get_parameter('visualization.rviz_arrow_thickness').value
        
                
        self.marker_lifetime = Duration(
            sec=int(lifetime_s), 
            nanosec=int((lifetime_s - int(lifetime_s)) * 1e9)
        )
        
        self.theoretical_collectors = {}
        
        self.cli_theory = self.create_client(Trigger, 'get_collector_theory')
        self.create_subscription(String, '/sim_status/collector_updates', self.update_theory_callback, 10)
        self.create_subscription(String, '/calibration/results', self.results_callback, 10)
        
        self.pub_markers = self.create_publisher(MarkerArray, '/calibration/rviz_markers', 10)
        
        self.frame_id = "world"  

        self.request_theoretical_map()
        self.get_logger().info("RViz Arrow Visualizer started [UNIFIED FACETS MODE]. Waiting for vectors...")

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
                    # Base kinematics (Yaw on Z, Pitch on Y)
                    rot_yaw = R.from_euler('z', c.get('yaw', 0.0))
                    rot_pitch = R.from_euler('y', c.get('pitch', 0.0))
                    global_c_r = rot_yaw * rot_pitch
                    
                    global_c_pos = np.array([c.get('x', 0.0), c.get('y', 0.0), c.get('z', 0.0)])
                    
                    if 'facets' in c:
                        for f in c['facets']:
                            local_offset = np.array(f.get('offset', [0.0, 0.0, 0.0]))
                            pos_f = global_c_pos + global_c_r.apply(local_offset)
                            
                            # Local facet kinematics (Roll and Pitch)
                            canting_rot = R.from_euler('xy', [f.get('cant_roll', 0.0), f.get('cant_pitch', 0.0)])
                            final_f_r = global_c_r * canting_rot
                            
                            # We store the PURE position and rotation
                            self.theoretical_collectors[str(f['id'])] = {
                                'pos': pos_f,
                                'rot': final_f_r
                            }
                    else:
                        self.theoretical_collectors[str(c['id'])] = {
                            'pos': global_c_pos,
                            'rot': global_c_r
                        }
                        
                self.get_logger().info(f"Arrow anchor points ready: {len(self.theoretical_collectors)} origins.")
        except Exception as e:
            self.get_logger().error(f"Error loading theory: {e}")

    def update_theory_callback(self, msg):
        self.request_theoretical_map()

    def results_callback(self, msg):
        if not self.theoretical_collectors:
            return 
            
        try:
            results = json.loads(msg.data)
            marker_array = MarkerArray()
            
            for p in results:
                c_id = str(p.get("id"))
                
                if c_id not in self.theoretical_collectors:
                    continue
                    
                # We extract from memory
                theo_c = self.theoretical_collectors[c_id]
                base_x, base_y, base_z = theo_c['pos']
                original_theo_r = theo_c['rot']
                
                # 1. Extract the already calculated basic vectors
                n_theo = p.get("theoretical_normal", [0.0, 0.0, 1.0])
                n_meas = p.get("measured_normal", [0.0, 0.0, 1.0])
                
                # 2. Extract the mean errors and convert them to radians
                error_x_rad = p.get("error_x_mrad", 0.0) / 1000.0
                error_y_rad = p.get("error_y_mrad", 0.0) / 1000.0
                
                # 3. RECONSTRUCT THE PERFECT MEAN NORMAL
                # We no longer have to invent the pitch and yaw, we use the real matrix
                r_error = R.from_euler('xy', [error_x_rad, error_y_rad])
                ccs_mean_n = r_error.apply([0.0, 0.0, 1.0])
                global_mean_n = original_theo_r.apply(ccs_mean_n)
                
                hash_id = abs(hash(str(c_id))) % 100000
                
                # Theoretical Arrow (BLUE)
                marker_theo = self.create_arrow(
                    marker_id=hash_id,
                    x=base_x, y=base_y, z=base_z,
                    vector=n_theo,
                    ns="1_theoretical_normal",
                    color=(0.0, 0.5, 1.0)
                )
                
                # Instantaneous Measured Arrow (RED)
                marker_meas = self.create_arrow(
                    marker_id=hash_id + 100000,
                    x=base_x, y=base_y, z=base_z,
                    vector=n_meas,
                    ns="2_instantaneous_measured_normal",
                    color=(1.0, 0.0, 0.2)
                )
                
                # Cumulative Mean Arrow (GREEN)
                marker_mean = self.create_arrow(
                    marker_id=hash_id + 200000,
                    x=base_x, y=base_y, z=base_z,
                    vector=global_mean_n.tolist(),
                    ns="3_cumulative_mean_normal",
                    color=(0.1, 0.9, 0.1) 
                )
                
                marker_array.markers.append(marker_theo)
                marker_array.markers.append(marker_meas)
                marker_array.markers.append(marker_mean)
                
            if marker_array.markers:
                self.pub_markers.publish(marker_array)
                
        except Exception as e:
            self.get_logger().error(f"Error processing results for RViz: {e}")

    def create_arrow(self, marker_id, x, y, z, vector, ns, color):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        p_start = Point(x=float(x), y=float(y), z=float(z))
        p_end = Point(
            x=float(x + (vector[0] * self.arrow_length)),
            y=float(y + (vector[1] * self.arrow_length)),
            z=float(z + (vector[2] * self.arrow_length))
        )
        marker.points = [p_start, p_end]
        
        # Scale: arrow thickness
        marker.scale.x = self.arrow_thick  
        marker.scale.y = self.arrow_thick * 2.0  
        marker.scale.z = self.arrow_thick * 2.0  
        
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color[0], color[1], color[2], 0.9
        
        marker.lifetime = self.marker_lifetime
        
        return marker

def main(args=None):
    rclpy.init(args=args)
    node = RVizCalibrationMarkersNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
