#!/usr/bin/env python3

import json
import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_sensor_data

class CameraFilterNode(Node):
    """
    2D Optical Filtering Node (Optimized with Vectorial NumPy).
    Applies the pinhole camera model to multiple impact points.
    """
    def __init__(self):
        super().__init__('camera_filter_node')
        
        # --- 1. DECLARAR PARÁMETROS DEL YAML ---
        self.declare_parameter('camera.focal_distance', 1.5)
        self.declare_parameter('camera.sensor_size', [1.6, 1.2])
        self.declare_parameter('camera.resolution', [640, 480])
        self.declare_parameter('camera.near_clip_dist', 0.1)

        # --- 2. EXTRAER VALORES ---
        self.focal_dist = self.get_parameter('camera.focal_distance').value
        
        sensor_dims = self.get_parameter('camera.sensor_size').value
        self.sensor_w, self.sensor_h = sensor_dims[0], sensor_dims[1]
        
        res_dims = self.get_parameter('camera.resolution').value
        self.res_w, self.res_h = res_dims[0], res_dims[1]
        
        self.near_clip = self.get_parameter('camera.near_clip_dist').value

        self.cam_pose = None

        # --- SUBSCRIPTIONS ---
        self.create_subscription(PoseStamped, '/data/camera', self.camera_callback, qos_profile_sensor_data)
        self.create_subscription(String, '/inspection/raw_data', self.raw_data_callback, qos_profile_sensor_data)
        
        # --- PUBLISHERS ---
        self.pub_filtered = self.create_publisher(String, '/inspection/filtered_data', 10)
        self.pub_log = self.create_publisher(String, '/sim_status/log', 10)

        self.get_logger().info("Vectorized Camera Filter active. Waiting for detections...")

    def camera_callback(self, msg):
        self.cam_pose = msg

    def raw_data_callback(self, msg):
        if not self.cam_pose: return

        try:
            impacts = json.loads(msg.data)
            if not impacts: return
        except:
            return

        # 1. We extract the camera kinematics (We use the Inverse just like in the Virtual Camera)
        p_c = np.array([self.cam_pose.pose.position.x, self.cam_pose.pose.position.y, self.cam_pose.pose.position.z])
        q_cam = self.cam_pose.pose.orientation
        r_cam_inv = R.from_quat([q_cam.x, q_cam.y, q_cam.z, q_cam.w]).inv()

        # 2. VECTORIZATION: We group all impacts
        world_points = np.array([imp.get('bounce_world_debug', [0, 0, 0]) for imp in impacts])
        
        # Geometric transformation using Scipy's safe apply function
        cam_points = r_cam_inv.apply(world_points - p_c)

        # 3. MASSIVE PROJECTION TO PIXELS
        # Mask 1: Only points that are in front of the lens (X > 0.1)
        front_mask = cam_points[:, 0] > self.near_clip
        
        visible = []
        
        # If there is at least 1 point in front, we apply the pinhole lens
        if np.any(front_mask):
            valid_indices = np.where(front_mask)[0]
            valid_p = cam_points[valid_indices]

            # Pinhole equations calculated all at once for the entire array
            y_p = -self.focal_dist * (valid_p[:, 1] / valid_p[:, 0])
            z_p = -self.focal_dist * (valid_p[:, 2] / valid_p[:, 0])

            u = ((y_p / self.sensor_w) + 0.5) * self.res_w
            v = ((z_p / self.sensor_h) + 0.5) * self.res_h

            # Mask 2: Screen boundaries (640x480)
            screen_mask = (u >= 0) & (u < self.res_w) & (v >= 0) & (v < self.res_h)

            # We filter the final indices that survived both masks
            final_indices = valid_indices[screen_mask]
            visible = [impacts[i] for i in final_indices]

        # 4. PUBLICATION AND LOG
        if visible:
            msg_final = String()
            msg_final.data = json.dumps(visible)
            self.pub_filtered.publish(msg_final)
            
            seen_ids = list(set([imp.get('collector_id', 'unknown') for imp in visible]))
            names = ", ".join(seen_ids)
            
            log = String()
            log.data = f"[FILTER] {len(visible)} impacts in FOV. Pieces: {names}"
            self.pub_log.publish(log)

def main(args=None):
    rclpy.init(args=args)
    node = CameraFilterNode()
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
