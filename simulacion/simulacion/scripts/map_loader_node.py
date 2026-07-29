#!/usr/bin/env python3

import csv
import json
import os
import math
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration

from scipy.spatial.transform import Rotation as R

# External injection scripts
try:
    from Add_collectors_from_file import inject_collectors
    from Remove_collectors_from_file import remove_collectors
except ImportError:
    def inject_collectors(m, p, mod): pass
    def remove_collectors(m, p): pass

# ==========================================
# GLOBAL CONFIGURATION
# ==========================================
# Change this to False to use the original simple collectors
ADVANCED_MODE = True 

# ==========================================
# MATHEMATICAL FUNCTIONS
# ==========================================

def get_invented_sun(date, time):
    return [1000.0, 100.0, 500.0]

def calculate_heliostat_orientation(p_c, p_aim, p_s):
    """
    The theoretical normal of each mirror will be the bisector
    between the direction connecting it to the sun and to 
    the point it should aim at.
    """
    v_dl = -np.array(p_s) + np.array(p_c)
    d_dl = v_dl / np.linalg.norm(v_dl)
    v_rl = np.array(p_aim) - np.array(p_c)
    d_rl = v_rl / np.linalg.norm(v_rl)
    n = d_rl - d_dl
    n = n / np.linalg.norm(n)
    yaw = np.arctan2(n[1], n[0])
    pitch = np.arcsin(n[2])
    return float(yaw), float(pitch)

# ==========================================
# MAP_LOADER_NODE
# ==========================================

class MapLoaderNode(Node):
    """
    Node for managing the collectors

    It includes the calculation of the theoretical direction for them and 
    the management of rotation orders given by the interface
    """
    def __init__(self):
        super().__init__('map_loader_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.theoretical_collectors = []   
        self.real_collectors = [] 
        
        self.current_world = "test1"
        self.current_model = "collector"

        # --- SUBSCRIBERS ---
        self.create_subscription(String, '/sim_cmd/map_management', self.map_management_callback, 10)
        self.create_subscription(String, '/sim_cmd/rotate_collector', self.rotate_collector_callback, 10)

        # --- PUBLISHERS ---
        self.pub_log = self.create_publisher(String, '/sim_status/log', 10)
        self.pub_updates = self.create_publisher(String, '/sim_status/collector_updates', 10)

        # --- SERVICES ---
        self.srv_theory = self.create_service(Trigger, 'get_collector_theory', self.get_collector_theory_callback)
        self.srv_real = self.create_service(Trigger, 'get_collector_real', self.get_collector_real_callback)
        
        # Timer to publish TF at 20Hz (to see in RViz)
        self.create_timer(0.05, self.broadcast_tf_tree)
        self.get_logger().info("Map Loader initialized with TF2 support.")

        mode_str = "ADVANCED (Facets)" if ADVANCED_MODE else "SIMPLE (Block)"
        self.send_log(f"Map Loader READY in {mode_str} MODE. Waiting for commands.")

    def broadcast_tf_tree(self):
        """
        Function to publish the tfs that allow visualizing 
        the reference systems in RVIZ.
        """
        now = self.get_clock().now().to_msg()
        for collector in self.real_collectors:
            # 1. FRAME: Collector pole (Yaw)
            # map -> collector_id
            t_collector = TransformStamped()
            t_collector.header.stamp = now
            t_collector.header.frame_id = 'world'
            t_collector.child_frame_id = f"{collector['id']}"
            t_collector.transform.translation.x = float(collector['x'])
            t_collector.transform.translation.y = float(collector['y'])
            t_collector.transform.translation.z = float(collector['z'])
            q_p = R.from_euler('z', collector['yaw']).as_quat()
            t_collector.transform.rotation.x, t_collector.transform.rotation.y = q_p[0], q_p[1]
            t_collector.transform.rotation.z, t_collector.transform.rotation.w = q_p[2], q_p[3]
            self.tf_broadcaster.sendTransform(t_collector)

            # 2. FRAME: Inclined plane of the collector (Pitch)
            # collector_id -> collector_inclined_id
            t_pitch = TransformStamped()
            t_pitch.header.stamp = now
            t_pitch.header.frame_id = f"{collector['id']}"
            t_pitch.child_frame_id = f"inclination_{collector['id']}"
            # We rotate on Y to apply the pitch of the entire collector
            q_pitch = R.from_euler('y', collector['pitch']).as_quat()
            t_pitch.transform.rotation.x, t_pitch.transform.rotation.y = q_pitch[0], q_pitch[1]
            t_pitch.transform.rotation.z, t_pitch.transform.rotation.w = q_pitch[2], q_pitch[3]
            self.tf_broadcaster.sendTransform(t_pitch)

            # 3. FRAME: Each facet (Offset + fine adjustment)
            # inclination_id -> facet_id
            """
            for facet in collector.get('facets', []):
                t_facet = TransformStamped()
                t_facet.header.stamp = now
                t_facet.header.frame_id = f"inclination_{collector['id']}"
                t_facet.child_frame_id = f"{facet['id']}"
                
                # Translation (Offset)
                t_facet.transform.translation.x = float(facet['offset'][0])
                t_facet.transform.translation.y = float(facet['offset'][1])
                t_facet.transform.translation.z = float(facet['offset'][2])
                
                # Local fine adjustment (Canting) on X (Roll) and Y (Pitch)
                q_f = R.from_euler('xy', [facet.get('cant_roll', 0.0), facet.get('cant_pitch', 0.0)]).as_quat()
                t_facet.transform.rotation.x, t_facet.transform.rotation.y = q_f[0], q_f[1]
                t_facet.transform.rotation.z, t_facet.transform.rotation.w = q_f[2], q_f[3]
                self.tf_broadcaster.sendTransform(t_facet)
            """
    def get_collector_theory_callback(self, request, response):
        response.success = True
        response.message = json.dumps(self.theoretical_collectors)
        return response

    def get_collector_real_callback(self, request, response):
        response.success = True
        response.message = json.dumps(self.real_collectors)
        return response

    def get_gazebo_entities(self, collector_list):
        """
        Translates our hierarchical memory into flat objects for Gazebo.
        - Global Collector: Yaw (Z vertical pole) + Pitch (Y horizontal hinge)
        - Facet: Local Roll (X axis) + Pitch (Y axis)
        """
        flat_entities = []
        
        for collector in collector_list:
            if not ADVANCED_MODE:
                flat_entities.append(collector)
                continue

            # 1. Base of the collector in the world (Translation vector)
            collector_pos = np.array([collector['x'], collector['y'], collector['z']])
            
            # 2. GLOBAL COLLECTOR KINEMATICS
            # We build the "motors" of the structure:
            rot_yaw = R.from_euler('z', collector['yaw'])      # Vertical pole motor
            rot_pitch = R.from_euler('y', collector['pitch'])  # Horizontal hinge motor
            
            # By multiplying (Yaw * Pitch), Scipy first applies the Pitch on the local Y, 
            # and then rotates the whole assembly on the vertical Z. 
            global_frame_rot = rot_yaw * rot_pitch

            for facet in collector.get('facets', []):
                # Displacement of the facet on the collector's grid
                local_offset = np.array(facet['offset'])

                # A) ABSOLUTE POSITION CALCULATION
                # We place the offset in the rotated frame and add it to the base
                absolute_pos = collector_pos + global_frame_rot.apply(local_offset)

                # B) LOCAL FACET ROTATION (Canting)
                # As you indicated, we keep this the same because facet rotations work fine
                cant_roll = facet.get('cant_roll', 0.0)
                cant_pitch = facet.get('cant_pitch', 0.0)
                canting_rot = R.from_euler('xy', [cant_roll, cant_pitch])
                
                # C) TOTAL ROTATION AND EXTRACTION
                # We add the rotation of the base structure + the rotation of the mirror itself
                absolute_rot = global_frame_rot * canting_rot

                # Classic extraction identical to the one TF used to pass to Gazebo
                final_euler = absolute_rot.as_euler('xyz')

                flat_entities.append({
                    "id": facet['id'],
                    "x": float(absolute_pos[0]),
                    "y": float(absolute_pos[1]),
                    "z": float(absolute_pos[2]),
                    "roll": float(final_euler[0]),
                    "pitch": float(final_euler[1]), 
                    "yaw": float(final_euler[2])    
                })

        return flat_entities
        
    def rotate_collector_callback(self, msg):
        try:
            data = json.loads(msg.data)
            target_id = data.get("collector_id")
            facet_id = data.get("facet_id", "all") 
            
            real_collector = next((c for c in self.real_collectors if c['id'] == target_id), None)
            
            if not real_collector:
                self.send_log(f"ERROR: The collector '{target_id}' does not exist.")
                return

            # =========================================================
            # SINGLE FACET MODE
            # =========================================================
            if ADVANCED_MODE and facet_id != "all":
                # 1. We look for the specific facet in memory before changing it
                old_facet = next((f for f in real_collector.get('facets', []) if f['id'] == facet_id), None)
                if not old_facet:
                    self.send_log(f"ERROR: Facet '{facet_id}' not found.")
                    return
                
                # 2. We calculate its state in Gazebo JUST BEFORE the change to delete ONLY that facet
                # To do this, we use a temporary list with a "fake collector" that only has that facet
                temp_collector = real_collector.copy()
                temp_collector['facets'] = [old_facet]
                single_old_entity = self.get_gazebo_entities([temp_collector])
                
                # We delete ONLY that facet in Gazebo
                remove_collectors(self.current_world, single_old_entity)

                # 3. We apply the rotation in memory to the facet
                old_facet['cant_roll'] += math.radians(data.get("roll_inc", 0.0))
                old_facet['cant_pitch'] += math.radians(data.get("pitch_inc", 0.0))

                # 4. We calculate the new position of ONLY that facet and inject it
                single_new_entity = self.get_gazebo_entities([temp_collector])
                inject_collectors(self.current_world, single_new_entity, "facet")

                self.send_log(f"SURGICAL [OK]: Only facet {facet_id} was replaced.")

            # =========================================================
            # GLOBAL MODE: MOVE THE ENTIRE COLLECTOR (ALL FACETS)
            # =========================================================
            else:
                # We delete the whole block/facets of the current collector
                old_entities = self.get_gazebo_entities([real_collector])
                remove_collectors(self.current_world, old_entities)

                # We modify the main tracking
                real_collector['yaw'] += math.radians(data.get("yaw_inc", 0.0))
                real_collector['pitch'] += math.radians(data.get("pitch_inc", 0.0))

                # We reinject the fully updated collector
                new_entities = self.get_gazebo_entities([real_collector])
                model_to_inject = "facet" if ADVANCED_MODE else self.current_model
                inject_collectors(self.current_world, new_entities, model_to_inject)
                
                self.send_log(f"GLOBAL ROTATION [OK]: {target_id} completely reorganized.")
            
            # 5. Notify the system
            self.pub_updates.publish(String(data=json.dumps([target_id])))
            
        except Exception as e:
            self.send_log(f"Rotation failure: {e}")
            
    def map_management_callback(self, msg):
        try:
            data = json.loads(msg.data)
            if data.get("action") == "LOAD":
                self.current_world = data.get("world", self.current_world)
                self.current_model = data.get("model", self.current_model)
                
                self.theoretical_collectors = self.generate_array_from_csv(
                    data.get("csv"), 
                    data.get("date", "10/02/2001"), 
                    data.get("time", "12:00")
                )
                self.real_collectors = json.loads(json.dumps(self.theoretical_collectors))
                
                # We translate to flat objects and send to Gazebo
                entities = self.get_gazebo_entities(self.real_collectors)
                model_to_inject = "facet" if ADVANCED_MODE else self.current_model
                inject_collectors(self.current_world, entities, model_to_inject)
                
                notification_ids = [c['id'] for c in self.real_collectors]
                self.pub_updates.publish(String(data=json.dumps(notification_ids)))
                self.send_log(f"MAP LOADED: {len(self.theoretical_collectors)} collectors (Advanced: {ADVANCED_MODE}).")

            elif data.get("action") == "EMPTY":
                entities = self.get_gazebo_entities(self.real_collectors)
                remove_collectors(self.current_world, entities)
                self.theoretical_collectors = []
                self.real_collectors = []
                self.pub_updates.publish(String(data="[]"))
                self.send_log("MAP EMPTIED.")

        except Exception as e:
            self.send_log(f"Management error: {e}")

    def generate_array_from_csv(self, csv_name, date, time_str):
        """
        We inspect the csv to find the theoretical data of the collectors
        """
        path = os.path.expanduser(f"~/{csv_name}")
        collector_list = []
        try:
            with open(path, mode='r', encoding='utf-8') as f:
                next(f) 
                reader = csv.DictReader(f)
                for row in reader:
                    if len(collector_list) >= 5: break 
                    
                    x, y, z = float(row["Heliostat x"]), float(row["Heliostat y"]), float(row["Heliostat z"])
                    ax, ay, az = float(row["Aiming point x"]), float(row["Aiming point y"]), float(row["Aiming point z"])
                    
                    yaw, pitch = calculate_heliostat_orientation([x,y,z], [ax,ay,az], get_invented_sun(date, time_str))
                    
                    collector_id = f"collector_{len(collector_list)}"
                    width_x = float(row["Heliostat width (x)"])
                    length_y = float(row["Heliostat length (y)"])
                    
                    # Base dictionary of the collector
                    collector_data = {
                        "id": collector_id, 
                        "x": x, "y": y, "z": z + 5,
                        "yaw": yaw, "pitch": pitch,
                        "width_x": width_x,
                        "length_y": length_y
                    }

                    # --- MODE 2: GENERATION OF 5x5 FACETS ---
                    if ADVANCED_MODE:
                        facets = []
                        w_facet = width_x / 5.0
                        l_facet = length_y / 5.0
                        
                        # Loop from -2 to +2 to generate a grid centered on the pole
                        for i in range(-2, 3):
                            for j in range(-2, 3):
                                facet_id = f"{collector_id}_f{i+2}_{j+2}"
                                facets.append({
                                    "id": facet_id,
                                    "offset": [i * w_facet, j * l_facet, 0.0],
                                    "cant_roll": 0.0,
                                    "cant_pitch": 0.0
                                })
                        collector_data["facets"] = facets
                    
                    collector_list.append(collector_data)
                    
            return collector_list
        except Exception as e:
            self.send_log(f"CSV Error: {e}")
            return []

    def send_log(self, text):
        msg = String(data=f"[MAP_LOADER] {text}")
        self.pub_log.publish(msg)
        self.get_logger().info(text)

def main(args=None):
    rclpy.init(args=args)
    node = MapLoaderNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__': main()
