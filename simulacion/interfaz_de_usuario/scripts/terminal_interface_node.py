import json
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TerminalInterfaceNode(Node):
    """
    Publisher node to send commands from the terminal.
    
    Publishes to various '/sim_cmd/' topics to configure and control
    the environment simulation.
    """

    def __init__(self):
        super().__init__('terminal_interface_node')
        
        self.declare_parameter('ui_defaults.world_name', 'test1')
        self.declare_parameter('ui_defaults.texture_name', 'none')
        self.declare_parameter('ui_defaults.date', '10/02/2001')
        self.declare_parameter('ui_defaults.time', '12:34')
        self.declare_parameter('ui_defaults.csv_path', 'Crescent_Dunes.csv')
        self.declare_parameter('ui_defaults.collector_model', 'collector')
        self.declare_parameter('ui_defaults.drone_model', 'x500')
        self.declare_parameter('ui_defaults.camera_pitch_deg', 45.0)
        self.declare_parameter('ui_defaults.target_collector', 'collector_0')
        self.declare_parameter('ui_defaults.drone_spawn', [0.0, 0.0, 0.5])
        
        # Publishers
        self.pub_world = self.create_publisher(String, '/sim_cmd/world_config', 10)
        self.pub_date = self.create_publisher(String, '/sim_cmd/date_config', 10)
        self.pub_collectors = self.create_publisher(String, '/sim_cmd/collector_config', 10)
        self.pub_drone = self.create_publisher(String, '/sim_cmd/drone_config', 10)
        self.pub_action = self.create_publisher(String, '/sim_cmd/action', 10)
        self.pub_camera = self.create_publisher(String, '/sim_cmd/rotate_camera', 10)
        self.pub_rotate_collector =self.create_publisher(String, '/sim_cmd/rotate_collector', 10)
 
    def publish_json(self, publisher, dictionary):
        """
        Converts a dictionary to JSON and publishes it as a String.
        """
        msg = String()
        msg.data = json.dumps(dictionary)
        publisher.publish(msg)

    def publish_action(self, action):
        """
        Publishes a plain text action to the corresponding topic.
        """
        msg = String()
        msg.data = action
        self.pub_action.publish(msg)

def interactive_menu(node):
    """
    Infinite loop that reads keyboard commands using input()
    and publishes the data through the node.
    """
    time.sleep(0.5)  # Short pause to ensure ROS 2 connects
    
    def_world = node.get_parameter('ui_defaults.world_name').value
    def_tex = node.get_parameter('ui_defaults.texture_name').value
    def_date = node.get_parameter('ui_defaults.date').value
    def_time = node.get_parameter('ui_defaults.time').value
    def_csv = node.get_parameter('ui_defaults.csv_path').value
    def_col_mod = node.get_parameter('ui_defaults.collector_model').value
    def_drone = node.get_parameter('ui_defaults.drone_model').value
    def_pitch = node.get_parameter('ui_defaults.camera_pitch_deg').value
    def_target = node.get_parameter('ui_defaults.target_collector').value
    def_spawn = node.get_parameter('ui_defaults.drone_spawn').value
    def_x = def_spawn[0]
    def_y = def_spawn[1]

    while True:
        print("\n" + "=" * 45)
        print("   SIMULATION CONTROL PANEL ")
        print("=" * 45)
        print("1.  Configure World (Name and Texture)")
        print("2.  Configure Date and Time")
        print("3.  Configure Collectors (CSV File and Model)")
        print("4.  Configure Drone (Model and Position)")
        print("-" * 45)
        print("5.  Rotate Camera live (Degrees)")
        print("6.  Rotate Collector live (Degrees)")
        print("-" * 45)        
        print("7.  Fill world")
        print("8.  Empty world")
        print("-" * 45)
        print("9.  LAUNCH SIMULATION (GENERATE)")
        print("10. Stop Simulation (TERMINATE)")
        print("11. Shutdown All and Exit (EXIT)")
        print("=" * 45)

        option = input(" Choose an option (1-11): ")

        if option == '1':
            name = input(f"   World name [{def_world}]: ") or def_world
            texture = input(f"   Texture path [{def_tex}]: ") or def_tex
            node.publish_json(node.pub_world, {"name": name, "texture": texture})
            print("   [OK] World data sent to the Orchestrator.")
            
        elif option == '2':
            date = input(f"   Date (e.g. 10/02/2001) [{def_date}]: ") or def_date
            time_str = input(f"   Time (e.g. 12:34) [{def_time}]: ") or def_time
            node.publish_json(node.pub_date, {"date": date, "time": time_str})
            print("   [OK] Date and time data sent to the Orchestrator.")
            
        elif option == '3':
            csv_path = input(f"   CSV path [{def_csv}]: ") or def_csv
            model = input(f"   Collector model [{def_col_mod}]: ") or def_col_mod
            node.publish_json(node.pub_collectors, {"csv_path": csv_path, "model": model})
            print("   [OK] Collector data sent to the Orchestrator.")

        elif option == '4':
            model = input(f"   Drone model [{def_drone}]: ") or def_drone
            try:
                x = float(input(f"   X Coordinate (e.g. 5.0) [{def_x}]: ") or str(def_x))
                y = float(input(f"   Y Coordinate (e.g. -2.0) [{def_y}]: ") or str(def_y))
                node.publish_json(node.pub_drone, {"model": model, "x": x, "y": y})
                print("   [OK] Drone data sent to the Orchestrator.")
            except ValueError:
                print("   [ERROR] Coordinates must be numbers. Try again.")
                
        elif option == '5':
            try:
                degrees = float(input(f"   Downward angle (0=Front, 90=Floor) [{def_pitch}]: ") or str(def_pitch))
                node.publish_json(node.pub_camera, {"angle": degrees})
                print(f"   [OK] Rotation command to {degrees}° sent.")
            except ValueError:
                print("   [ERROR] Enter a valid number.")
                
        elif option == '6':
            try:
                input_id = input(f"   Collector or facet ID to rotate [{def_target}]: ") or def_target
                
                if "_f" in input_id:
                    roll_angle = float(input("   roll increment [0.0]: ") or "0.0")
                    pitch_angle = float(input("   pitch increment [0.0]: ") or "0.0")
                    collector_id = input_id.split("_f")[0]
                    facet_id = input_id
                    
                    node.publish_json(node.pub_rotate_collector, {
                        "collector_id": collector_id, 
                        "facet_id": facet_id,
                        "roll_inc": roll_angle, 
                        "pitch_inc": pitch_angle
                    })
                else:
                    yaw_angle = float(input("   yaw increment [0.0]: ") or "0.0")
                    pitch_angle = float(input("   pitch increment [0.0]: ") or "0.0")
                    collector_id = input_id
                    facet_id = "all"
                
                    node.publish_json(node.pub_rotate_collector, {
                        "collector_id": collector_id, 
                        "facet_id": facet_id,
                        "yaw_inc": yaw_angle, 
                        "pitch_inc": pitch_angle
                    })
                
                if facet_id == "all":
                    print(f"   [OK] Command sent to full collector {collector_id} for yaw += {yaw_angle}° and pitch += {pitch_angle}º.")
                else:
                    print(f"   [OK] Command sent to facet {facet_id} of {collector_id} for roll += {roll_angle}° and pitch += {pitch_angle}º.")
                    
            except ValueError:
                print("   [ERROR] Enter a valid ID and numerical angle values.")
                
        elif option == '7':
            print("\n   >>  Sending FILL command...")
            node.publish_action("FILL")
        elif option == '8':
            print("\n   >>  Sending EMPTY command...")
            node.publish_action("EMPTY")
        elif option == '9':
            print("\n   >>  Sending GENERATE command...")
            node.publish_action("GENERATE")
        elif option == '10':
            print("\n   >>  Sending TERMINATE command...")
            node.publish_action("TERMINATE")
        elif option == '11':
            print("\n   >>  Shutting down the Orchestrator and exiting...")
            node.publish_action("EXIT")
            break
        else:
            print("   [!] Invalid option. Enter a number between 1 and 11.")


def main(args=None):
    rclpy.init(args=args)
    node = TerminalInterfaceNode()

    # SEPARATION INTO TWO THREADS:
    # ROS 2 spins in the background to allow sending messages
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    # The main thread gets trapped in the input() waiting for the user
    try:
        interactive_menu(node)
    except KeyboardInterrupt:
        print("\nExiting the panel via keyboard (Ctrl+C).")
    finally:
        # Ensures the node is cleaned up and ROS 2 is closed properly upon exit
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
