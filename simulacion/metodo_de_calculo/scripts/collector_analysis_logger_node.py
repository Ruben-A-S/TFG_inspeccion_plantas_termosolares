#!/usr/bin/env python3

import os
import csv
import json
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

class CollectorAnalysisLoggerNode(Node):
    """
    Multi-facet Inspection Dashboard (Real-Time Mode).
    Shows the current state of the facets with an implicit sliding window
    upon receiving processed data from the CalibrationNode.
    """
    def __init__(self):
        super().__init__('collector_analysis_logger_node')
        
        self.rows = 5
        self.cols = 5
        
        self.csv_filename = 'collector_inspection_history.csv'
        file_exists = os.path.isfile(self.csv_filename)
        self.csv_file = open(self.csv_filename, mode='a', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        if not file_exists:
            self.csv_writer.writerow(['Timestamp', 'Collector_ID', 'Facet_ID', 'Err_X_mrad', 'Err_Y_mrad'])
            
        self.bridge = CvBridge()
        
        # Main current state dictionary: { "collector_0": { "visits", "err_x", "err_y" } }
        self.plant_data = {}  
        self.active_collector = None 
        
        self.create_subscription(String, '/calibration/results', self.results_callback, 10)
        self.pub_dashboard = self.create_publisher(Image, '/calibration/heatmap_image', 10)
        
        self.timer_dashboard = self.create_timer(1.5, self.publish_dashboard)
        self.get_logger().info("Real-time Logger started. 5x5 Dashboard refreshing...")

    def extract_facet_indices(self, facet_id):
        try:
            parts = facet_id.split('_f')
            if len(parts) != 2: return facet_id, None, None
            
            parent_id = parts[0]
            indices = parts[1].split('_')
            return parent_id, int(indices[1]), int(indices[0])
        except:
            return facet_id, None, None

    def results_callback(self, msg):
        try:
            results = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        timestamp = str(self.get_clock().now().to_msg().sec)

        for data in results:
            full_id = data.get("id")
            if not full_id: continue

            parent_id, c, f = self.extract_facet_indices(full_id)
            if c is None or f is None: continue

            self.active_collector = parent_id
            if parent_id not in self.plant_data:
                self.plant_data[parent_id] = {
                    'visits': np.zeros((self.rows, self.cols)),
                    'err_x': np.zeros((self.rows, self.cols)),
                    'err_y': np.zeros((self.rows, self.cols))
                }

            # Instant update (We overwrite the most recent value)
            err_x = data.get("error_x_mrad", 0.0)
            err_y = data.get("error_y_mrad", 0.0)
            
            self.plant_data[parent_id]['err_x'][f, c] = err_x
            self.plant_data[parent_id]['err_y'][f, c] = err_y
            self.plant_data[parent_id]['visits'][f, c] = 1 # Mark as active
            
            self.csv_writer.writerow([timestamp, parent_id, full_id, err_x, err_y])
        self.csv_file.flush()

    def publish_dashboard(self):
        if self.active_collector is None or self.active_collector not in self.plant_data:
            return

        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
        fig.suptitle(f'Real-Time Dashboard - {self.active_collector}', fontsize=16, fontweight='bold')
        
        data = self.plant_data[self.active_collector]
        mask = data['visits'] > 0

        for ax in [ax1, ax2]:
            ax.set_xticks(np.arange(self.cols + 1) - 0.5, minor=True)
            ax.set_yticks(np.arange(self.rows + 1) - 0.5, minor=True)
            ax.grid(which="minor", color="black", linestyle='-', linewidth=2)
            ax.tick_params(which="minor", size=0)
            ax.invert_yaxis()
            ax.set_aspect('equal')

        # 1. Coverage
        ax1.set_title("Coverage (Inspected)", pad=15)
        ax1.imshow(data['visits'], cmap='Greens', origin='upper', vmin=0, vmax=1)

        # 2. Vector Map
        ax2.set_title("Normal Deviation (Canting Error)", pad=15)
        ax2.imshow(np.zeros((self.rows, self.cols)), cmap='gray', alpha=0.1, origin='upper')

        X, Y = np.meshgrid(np.arange(self.cols), np.arange(self.rows))
        U = np.where(mask, -data['err_x'], 0)
        V = np.where(mask, data['err_y'], 0)
        mag = np.sqrt(U**2 + V**2)
        max_mag = np.max(mag)
        
        ax2.scatter(X[mask], Y[mask], color='dimgray', s=20)
        if np.max(mag) > 0.01:
            # 1. DEFINE THRESHOLD (in mrad)
            # Any error below this will be drawn very small.
            fixed_threshold_mrad = 5.0
            
            # 2. HYBRID LOGIC
            # If max_mag is 0.5, reference_value will be 5.0 (Fixed Scale)
            # If max_mag is 15.0, reference_value will be 15.0 (Dynamic Scale)
            reference_value = max(max_mag, fixed_threshold_mrad)
            
            # 3. APPLY SCALE
            # We make the reference value occupy at most 90% of the cell
            hybrid_scale = reference_value / 0.9
            
            q = ax2.quiver(X[mask], Y[mask], U[mask], V[mask], mag[mask], 
                           cmap='jet',            # Rainbow color palette
                           pivot='mid',
                           angles='xy',           # Respects the 2D matrix
                           scale_units='xy',      # Uses grid units
                           scale=hybrid_scale,    # Applied smart scale
                           width=0.015,           # Thickness
                           headwidth=5,
                           headlength=6)
            
            # We force the colorbar limits so it doesn't "dance" 
            # if the values are very small
            q.set_clim(vmin=0, vmax=reference_value)
            
            fig.colorbar(q, ax=ax2, label='Error Magnitude (mrad)')

        fig.tight_layout()
        fig.canvas.draw()
        img = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8).reshape(fig.canvas.get_width_height()[::-1] + (3,))
        plt.close(fig) 
        
        self.pub_dashboard.publish(self.bridge.cv2_to_imgmsg(img, encoding="rgb8"))

def main(args=None):
    rclpy.init(args=args)
    node = CollectorAnalysisLoggerNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.csv_file.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
