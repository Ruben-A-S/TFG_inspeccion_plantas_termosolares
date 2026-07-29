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
            name = input("   World name [test1]: ") or "test1"
            texture = input("   Texture path [none]: ") or "none"
            node.publish_json(node.pub_world, {"name": name, "texture": texture})
            print("   [OK] World data sent to the Orchestrator.")
            
        elif option == '2':
            date = input("   Date (e.g. 10/02/2001) [10/02/2001]: ") or "10/02/2001"
            time_str = input("   Time (e.g. 12:34) [12:34]: ") or "12:34"
            node.publish_json(node.pub_date, {"date": date, "time": time_str})
            print("   [OK] Date and time data sent to the Orchestrator.")
            
        elif option == '3':
            csv_path = input("   CSV path [Crescent_Dunes.csv]: ") or "Crescent_Dunes.csv"
            model = input("   Collector model [collector]: ") or "collector"
            node.publish_json(node.pub_collectors, {"csv_path": csv_path, "model": model})
            print("   [OK] Collector data sent to the Orchestrator.")

        elif option == '4':
            model = input("   Drone model [x500]: ") or "x500"
            try:
                x = float(input("   X Coordinate (e.g. 5.0) [0.0]: ") or "0.0")
                y = float(input("   Y Coordinate (e.g. -2.0) [0.0]: ") or "0.0")
                node.publish_json(node.pub_drone, {"model": model, "x": x, "y": y})
                print("   [OK] Drone data sent to the Orchestrator.")
            except ValueError:
                print("   [ERROR] Coordinates must be numbers. Try again.")
                
        elif option == '5':
            try:
                degrees = float(input("   Downward angle (0=Front, 90=Floor) [45]: ") or "45")
                node.publish_json(node.pub_camera, {"angle": degrees})
                print(f"   [OK] Rotation command to {degrees}° sent.")
            except ValueError:
                print("   [ERROR] Enter a valid number.")
                
        elif option == '6':
            try:
                input_id = input("   Collector or facet ID to rotate [collector_0]: ") or "collector_0"
                
                if "_f" in input_id:
                    # If the user types "collector_4_f4_0", we split it by "_f"
                    roll_angle = float(input("   roll increment [0.0]: ") or "0.0")
                    pitch_angle = float(input("   pitch increment [0.0]: ") or "0.0")
                    # The parent will be "collector_4" and the facet will be the full text
                    collector_id = input_id.split("_f")[0]
                    facet_id = input_id
                    
                    # We publish the JSON with the two fields separated correctly
                    node.publish_json(node.pub_rotate_collector, {
                        "collector_id": collector_id, 
                        "facet_id": facet_id,
                        "roll_inc": roll_angle, 
                        "pitch_inc": pitch_angle
                    })
                    
                else:
                    # If they type just "collector_4", the whole block rotates
                    yaw_angle = float(input("   yaw increment [0.0]: ") or "0.0")
                    pitch_angle = float(input("   pitch increment [0.0]: ") or "0.0")
                    collector_id = input_id
                    facet_id = "all"
                
                    # We publish the JSON with the two fields separated correctly
                    node.publish_json(node.pub_rotate_collector, {
                        "collector_id": collector_id, 
                        "facet_id": facet_id,
                        "yaw_inc": yaw_angle, 
                        "pitch_inc": pitch_angle
                    })
                
                # Dynamic confirmation message
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
