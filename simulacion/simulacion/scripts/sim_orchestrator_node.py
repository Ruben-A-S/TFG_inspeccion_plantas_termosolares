import json
import os
import subprocess
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

# We import the external world generator script
from world_generator import create_base_world


class SimOrchestratorNode(Node):
    """
    Simulation Orchestrator Node.
    
    It is responsible for storing the configuration sent by the user,
    generating the world, launching PX4 SITL with Gazebo, and sending the
    load/empty solar collector commands to the corresponding nodes.
    """

    def __init__(self):
        super().__init__('sim_orchestrator_node')

        # --- INTERNAL STATE ---
        self.config_date = {"date": "10/02/2001", "time": "12:34"}
        self.config_world = {"name": "test1", "texture": "arenosillo.png"}
        self.config_collectors = {"model": "collector", "csv_path": "Crescent_Dunes.csv"}
        self.config_drone = {"model": "x500", "x": 0.0, "y": 0.0}
        self.simulation_process = None 

        self.generated_world = {"name": "test1"}
        self.generated_collectors = {"csv_path": "mapa_3.txt"}
        
        # --- PUBLISHERS ---
        self.pub_map_management = self.create_publisher(String, '/sim_cmd/map_management', 10)
        
        self.pub_state = self.create_publisher(String, '/sim_status/state', 10)
        
        self.pub_log = self.create_publisher(String, '/sim_status/log', 10)
        
        self.pub_active_sim = self.create_publisher(String, '/sim_status/active_sim', 10)
        
        self.pub_control_params = self.create_publisher(Float64MultiArray, '/control_param', 10)
        
        # --- SUBSCRIBERS ---
        self.create_subscription(String, '/sim_cmd/date_config', self.config_date_callback, 10)
        
        self.create_subscription(String, '/sim_cmd/world_config', self.config_world_callback, 10)
        
        self.create_subscription(String, '/sim_cmd/collector_config', self.config_collectors_callback, 10)
        
        self.create_subscription(String, '/sim_cmd/drone_config', self.config_drone_callback, 10)
        
        self.create_subscription(String, '/sim_cmd/action', self.action_callback, 10)
        
        self.create_subscription(String, '/sim_cmd/rotate_camera', self.rotate_camera_callback, 10)
        
        self.create_subscription(String, '/sim_cmd/rotate_collector', self.rotate_collector_callback, 10)

        self.send_log("Orchestrator Node Started. Waiting for configurations...")
        self.change_state("WAITING_FOR_DATA")

    # ==========================================
    # DATA CALLBACKS
    # ==========================================
    
    def config_date_callback(self, msg):
        """Updates internal date and time."""
        try:
            self.config_date = json.loads(msg.data)
            date_str = self.config_date.get('date')
            time_str = self.config_date.get('time')
            self.send_log(f"Date configuration updated: {date_str} at {time_str}")
        except json.JSONDecodeError:
            self.send_log("ERROR: Invalid date JSON.")
            
    def config_world_callback(self, msg):
        """Updates the name and texture of the world to generate."""
        try:
            self.config_world = json.loads(msg.data)
            name = self.config_world.get('name')
            texture = self.config_world.get('texture')
            self.send_log(f"World configuration updated: {name} (texture: {texture})")
        except json.JSONDecodeError:
            self.send_log("ERROR: Invalid world JSON.")

    def config_collectors_callback(self, msg):
        """Updates the collector generation configuration."""
        try:
            self.config_collectors = json.loads(msg.data)
            csv_path = self.config_collectors.get('csv_path')
            model = self.config_collectors.get('model')
            self.send_log(f"Collector configuration updated: {csv_path} (model: {model})")
        except json.JSONDecodeError:
            self.send_log("ERROR: Invalid collector JSON.")

    def config_drone_callback(self, msg):
        """Updates the drone model and its takeoff position."""
        try:
            self.config_drone = json.loads(msg.data)
            model = self.config_drone.get('model')
            pos_x = self.config_drone.get('x')
            pos_y = self.config_drone.get('y')
            self.send_log(f"Drone configuration updated: {model} at X={pos_x}, Y={pos_y}")
        except json.JSONDecodeError:
            self.send_log("ERROR: Invalid drone JSON.")
    
    def rotate_camera_callback(self, msg):
        """Updates the drone camera pitch."""
        try:
            data = json.loads(msg.data)
            degrees = data.get("angle", 45.0)
            
            # Convert to radians here so the calculator receives the ready-to-use data
            radians = degrees * (3.14159265 / 180.0)
            
            # Prepare the message for the calculator (Float64MultiArray)
            msg_control = Float64MultiArray()
            # [Angle, Focal (default 1.5), Distortion (0.0)]
            msg_control.data = [float(radians), 1.5, 0.0]        
            self.pub_control_params.publish(msg_control)
            
            self.send_log(f"Camera moved to {degrees} degrees ({radians:.3f} rad)")
            
        except Exception as e:
            self.send_log(f"ERROR processing camera angle: {e}")
    
    def rotate_collector_callback(self, msg):
        """Listens to the rotate collector command to register it in the global log."""
        try:
            data = json.loads(msg.data)
            c_id = data.get("collector_id", "collector_0") 
            
            # Read the facet so the log is exact
            f_id = data.get("facet_id", "all") 
            
            if "_f" in f_id:
                roll_inc_degrees = data.get("roll_inc", 0.0)
                pitch_inc_degrees = data.get("pitch_inc", 0.0)
            
                # Update text to reflect if we rotate everything or just one piece
                self.send_log(
                    f"Command received: Rotate {c_id} (Facet: {f_id}) "
                    f"(Roll: +{roll_inc_degrees}º, Pitch: +{pitch_inc_degrees}º). "
                    f"Delegating execution to the Map Manager."
                )
                
            else:
                yaw_inc_degrees = data.get("yaw_inc", 0.0)
                pitch_inc_degrees = data.get("pitch_inc", 0.0)
            
                # Update text to reflect if we rotate everything or just one piece
                self.send_log(
                    f"Command received: Rotate {c_id} (Facet: {f_id}) "
                    f"(Yaw: +{yaw_inc_degrees}º, Pitch: +{pitch_inc_degrees}º). "
                    f"Delegating execution to the Map Manager."
                )
            
        except Exception as e:
            self.send_log(f"ERROR processing collector rotation in the Orchestrator: {e}")
            
    # ==========================================
    # MAIN ACTIONS CALLBACK
    # ==========================================
    
    def action_callback(self, msg):
        """Receives a main action command (GENERATE, POPULATE, etc.)."""
        command = msg.data.upper()
        
        if command == "GENERATE":
            self.execute_full_generation()
        elif command == "FILL":
            self.inject_collectors()
        elif command == "EMPTY":
            self.remove_collectors()
        elif command == "TERMINATE":
            self.close_simulation()
        elif command == "EXIT":
            self.send_log("Total exit command received. Cleaning up...")
            self.close_simulation()
            raise SystemExit  
        else:
            self.send_log(f"Unknown command: {command}")

    # ==========================================
    # WORLD GENERATION
    # ==========================================
    
    def execute_full_generation(self):
        """Starts Gazebo and PX4 SITL with the current configurations."""
        if self.simulation_process is not None:
            self.send_log(
                "WARNING: The simulation is already running. "
                "Close the current one (Option 10) before generating another."
            )
            return
            
        self.change_state("STARTING_SIMULATION")
        self.send_log("Phase 1: Preparing virtual world...")
        
        world_name = self.config_world.get('name', 'test1')
        texture_name = self.config_world.get('texture', 'arenosillo.png')
        
        # Hardcoded paths (could be extracted to ROS 2 parameters in the future)
        base_dir = os.path.expanduser("~/Carpeta_TFG_Provisional/src/TFG_inspeccion_plantas_termosolares")
        original_world_path = os.path.join(base_dir, "simulacion/simulacion/worlds", f"{world_name}.sdf")
        texture_path = os.path.join(base_dir, "simulacion/simulacion/models/textures", texture_name)
        
        try:
            create_base_world(world_name, texture_path, original_world_path)
            self.send_log(f"World '{world_name}' generated successfully.")
        except Exception as e:
            self.send_log(f"ERROR generating the world: {e}")
            return 
            
        self.send_log("Phase 2: Preparing paths for PX4...")
        
        drone_model = self.config_drone.get('model', 'x500')
        pos_x = self.config_drone.get('x', 0.0)
        pos_y = self.config_drone.get('y', 0.0)

        px4_worlds_path = os.path.expanduser("~/PX4-Autopilot/Tools/simulation/gz/worlds")
        destination_world_path = os.path.join(px4_worlds_path, f"{world_name}.sdf")

        if os.path.exists(original_world_path):
            self.send_log("Copying world to PX4 environment...")
            subprocess.run(f"cp {original_world_path} {destination_world_path}", shell=True)
        else:
            self.send_log(f"WARNING: File {original_world_path} was not found.")

        command = (
            f"export PX4_GZ_WORLD={world_name} && "
            f"export PX4_GZ_MODEL_POSE='{pos_x},{pos_y},0.5,0,0,0' && "
            f"cd ~/PX4-Autopilot && make px4_sitl gz_{drone_model}"
        )
        
        self.send_log("Phase 3: Launching simulation...")
        self.simulation_process = subprocess.Popen(
            command, shell=True, executable='/bin/bash'
        )
        
        self.change_state("SIMULATION_RUNNING")
        self.generated_world = {"name": world_name}
        
        active_config = {
            "world": world_name,
            "drone": drone_model
        }
        msg_active = String()
        msg_active.data = json.dumps(active_config)
        self.pub_active_sim.publish(msg_active)
        
    def close_simulation(self):
        """Kills Gazebo and PX4 processes."""
        self.send_log("Closing simulator and cleaning up Linux processes...")
        # Uses stderr=subprocess.DEVNULL to hide errors if there are no processes to kill
        subprocess.run("killall -9 ruby px4 gz", shell=True, stderr=subprocess.DEVNULL)
        self.simulation_process = None
        self.change_state("WAITING_FOR_DATA")
        self.send_log("Simulator closed.")

    # ==========================================
    # COLLECTOR MANAGEMENT
    # ==========================================
    
    def inject_collectors(self):
        """Asks the map loading node to insert the solar collectors."""
        world_date = self.config_date.get('date', '10/02/2001')
        world_time = self.config_date.get('time', '12:34')
        csv_name = self.config_collectors.get('csv_path', 'mapa_3.txt')
        collector_model = self.config_collectors.get('model', 'collector')
        world_name = self.generated_world.get('name', 'test1')
        
        # 1. Update internal state
        self.generated_collectors = {"csv_path": csv_name}
        
        # 2. Package the command
        command = {
            "action": "LOAD",
            "date": world_date,
            "time": world_time,
            "csv": csv_name,
            "model": collector_model,
            "world": world_name
        }
        
        # 3. Send the command to the load_map node
        msg = String()
        msg.data = json.dumps(command)
        self.pub_map_management.publish(msg)
        
        self.send_log(f"Command sent to load_map to fill '{world_name}' with '{csv_name}'.")
        
    def remove_collectors(self):
        """Asks the map loading node to remove the solar collectors."""
        csv_name = self.generated_collectors.get('csv_path', 'mapa_3.txt')
        world_name = self.generated_world.get('name', 'test1')

        command = {
            "action": "EMPTY",
            "csv": csv_name,
            "world": world_name
        }

        msg = String()
        msg.data = json.dumps(command)
        self.pub_map_management.publish(msg)
        
        self.send_log(f"Command sent to load_map to empty the map '{csv_name}'.")
        self.generated_collectors = {}

    # ==========================================
    # UTILITIES
    # ==========================================
    
    def send_log(self, text):
        """Publishes a message to the logs topic and prints it locally."""
        msg = String()
        msg.data = text
        self.pub_log.publish(msg)
        self.get_logger().info(text)

    def change_state(self, new_state):
        """Updates the global simulation state."""
        msg = String()
        msg.data = new_state
        self.pub_state.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimOrchestratorNode()
    
    try:
        rclpy.spin(node)
    except SystemExit:
        node.get_logger().info("Node shutdown requested by the user. Goodbye.")
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown via Ctrl+C.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
