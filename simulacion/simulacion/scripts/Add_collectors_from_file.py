import math
import os
import subprocess


def euler_to_quaternion(roll, pitch, yaw):
    """
    Converts Euler angles (radians) to Quaternions (x, y, z, w).
    
    Required by Gazebo to define the spatial orientation of a model.
    """
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy

    return qx, qy, qz, qw


def inject_collectors(world, collector_array, model, base_dir, timeout_ms=3000):
    """
    Reads an array of collectors and injects each model into the active Gazebo world.
    
    Uses the terminal command 'gz service' to spawn the model
    with the specified position and orientation (translated to quaternions).
    """
    
    model_path = os.path.join(
        base_dir, f"simulacion/simulacion/models/{model}.sdf"
    )
    
    for collector in collector_array:
        # Extract data from the dictionary
        collector_id = collector['id']
        x = collector['x']
        y = collector['y']
        z = collector['z']
        roll = collector.get('roll', 0.0)
        pitch = collector.get('pitch', 0.0)
        yaw = collector.get('yaw', 0.0)
        
        # Convert angles for Gazebo 
        # (Roll is always 0 for a mirror resting on the ground)
        qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)
        
        # Injection command via Gazebo CLI
        command = (
            f"gz service -s /world/{world}/create "
            f"--reqtype gz.msgs.EntityFactory "
            f"--reptype gz.msgs.Boolean "
            f"--timeout {timeout_ms} "
            f"--req 'sdf_filename: \"{model_path}\", name: \"{collector_id}\", "
            f"pose: {{position: {{x: {x}, y: {y}, z: {z}}}, "
            f"orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}}}'"
        )
        
        # Execute silently in the Linux terminal
        subprocess.run(
            command, 
            shell=True, 
            stdout=subprocess.DEVNULL, 
            stderr=subprocess.DEVNULL
        )
