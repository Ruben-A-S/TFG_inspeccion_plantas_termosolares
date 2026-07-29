#!/usr/bin/env python3

import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# We define the EXACT communication profile to avoid QoS conflicts
my_qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=10
)

class Color:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'

class FeedbackMonitorNode(Node):
    """
    Subscriber node for different topics that offers 
    feedback on the state of the simulation.

    In addition to showing feedback from commands sent via the interface, 
    it allows visualization of the inspection method's results, 
    displaying the measured orientation error.
    """
    
    def __init__(self):
        super().__init__('feedback_monitor_node')
        
        self.last_print = {} 
        self.refresh_interval = 1.0 

        # --- SUBSCRIPTIONS ---
        self.create_subscription(String, '/sim_status/log', self.log_callback, 10)
        self.create_subscription(String, '/sim_status/state', self.state_callback, 10)
        self.create_subscription(String, '/sim_status/collector_updates', self.updates_callback, 10)
        self.create_subscription(String, '/calibration/results', self.results_callback, 10)
        
        # Subscription to the Faker with the forced QoS profile to avoid the WARNING
        self.create_subscription(String, '/inspection/raw_data', self.raw_data_callback, my_qos_profile)
        
        self.current_state = ""

        print(f"\n{Color.OKCYAN}{Color.BOLD}" + "=" * 45)
        print("          ACTIVE SOLAR THERMAL INSPECTION CENTRAL MONITOR")
        print("=" * 45 + f"{Color.ENDC}\n")

    def state_callback(self, msg):
        new_state = msg.data
        if new_state != self.current_state:
            self.current_state = new_state
            print(f"{Color.BOLD}{Color.OKBLUE}[SYSTEM]{Color.ENDC} State: {Color.BOLD}{new_state}{Color.ENDC}")

    def updates_callback(self, msg):
        try:
            ids = json.loads(msg.data)
            if ids:
                print(f"{Color.OKCYAN}[UPDATE]{Color.ENDC} Modified entities: {ids}")
            else:
                print(f"{Color.WARNING}[UPDATE]{Color.ENDC} Map emptied.")
        except:
            pass

    def raw_data_callback(self, msg):
        try:
            impacts = json.loads(msg.data)
            now = time.time()
            
            for imp in impacts:
                # Changed "id_panel" to "collector_id" to match the first script
                c_id = imp.get("collector_id", "Unknown")
                spam_key = f"{c_id}_raw"
                
                if now - self.last_print.get(spam_key, 0.0) > self.refresh_interval:
                    self.last_print[spam_key] = now
                    
                    bounce_w = imp.get("bounce_world_debug", [0, 0, 0])
                    bounce_l = imp.get("bounce_local", [0, 0, 0])
                    
                    ctype = "FACET" if "_f" in c_id else "COLLECTOR"
                    
                    print(f"{Color.WARNING}{Color.BOLD}[IMPACT]{Color.ENDC} "
                          f"{ctype}: {Color.BOLD}{c_id}{Color.ENDC} | "
                          f"Global: ({bounce_w[0]:.1f}, {bounce_w[1]:.1f}, {bounce_w[2]:.1f})")
        except:
            pass

    def results_callback(self, msg):
        try:
            res = json.loads(msg.data)
            print(f"\n{Color.OKGREEN}{Color.BOLD}=== HELIOPOINT CALIBRATION REPORT ==={Color.ENDC}")
            
            for p in res:
                c_id = p.get('id', 'Unknown')
                samples = p.get('samples_taken', 0)
                error_x = p.get('error_x_mrad', 0.0)
                error_y = p.get('error_y_mrad', 0.0)
                
                ctype = "FACET" if "_f" in c_id else "COLLECTOR"
                print(f"{Color.OKGREEN}{Color.BOLD}[{ctype} {c_id}]{Color.ENDC} Samples: {samples}")
                print(f" ├─ Mean Error -> X: {error_x:.2f} mrad | Y: {error_y:.2f} mrad")
            
            print(f"{Color.OKGREEN}{Color.BOLD}========================================={Color.ENDC}\n")
                
        except Exception as e:
            print(f"{Color.FAIL}[ERROR] Failed to read calibration results: {e}{Color.ENDC}")
    
    def log_callback(self, msg):
        text = msg.data
        if any(word in text.upper() for word in ["ERROR", "FAIL"]):
            print(f"{Color.FAIL}{Color.BOLD}[ERROR!] {Color.ENDC}{Color.FAIL}{text}{Color.ENDC}")
        elif any(word in text.upper() for word in ["SUCCESS", "OK"]):
            print(f"{Color.OKGREEN}{Color.BOLD}[OK] {Color.ENDC}{text}")

def main(args=None):
    rclpy.init(args=args)
    node = FeedbackMonitorNode()
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
