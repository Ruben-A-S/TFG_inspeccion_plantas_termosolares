#!/usr/bin/env python3

import os
import subprocess
import sys


def _kill_collector(world, name):
    """
    Internal function that executes a bash script to remove a Gazebo model.
    """
    print(f"Python ordering removal of: {name} in world: {world}...")
    
    # Get the absolute path of the .sh script
    current_dir = os.path.dirname(os.path.abspath(__file__))
    bash_path = os.path.join(current_dir, "remove_collector.sh")

    # Prepare the command
    command = [
        bash_path, 
        str(world), 
        str(name)
    ]
    
    # Execute and capture the output so it doesn't clutter the terminal unless we want it to
    result = subprocess.run(command, capture_output=True, text=True)
    
    if result.returncode == 0:
        print(f"  [OK] {name} successfully removed!")
    else:
        print(f"  [ERROR] Error removing {name}: {result.stderr}")


def remove_collectors(world_name, collector_array):
    """
    Iterates through a list of collectors and removes them one by one from the Gazebo environment.
    
    :param world_name: String with the name of the Gazebo world.
    :param collector_array: List of dictionaries, where each collector has an 'id'.
    """
    for collector in collector_array:
        # Extract the ID, which is the only thing Gazebo needs to delete it
        collector_id = collector.get('id')
        
        if collector_id:
            _kill_collector(world_name, collector_id)
            
    print("[SUCCESS] Collector removal finished.")
    return True


if __name__ == "__main__":
    # --- PROTECTION FOR INDEPENDENT TERMINAL USE ---
    if len(sys.argv) < 3:
        print("Usage: python3 collector_remover.py <world> <object_name>")
        sys.exit(1)
        
    world_arg = sys.argv[1]
    name_arg = sys.argv[2]   
    
    # If used via terminal, we simulate an array with a single collector
    remove_collectors(world_arg, [{"id": name_arg}])
