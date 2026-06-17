import math
import os
import subprocess


def euler_a_cuaternion(roll, pitch, yaw):
    """
    Convierte ángulos de Euler (radianes) a Cuaterniones (x, y, z, w).
    
    Requerido por Gazebo para definir la orientación espacial de un modelo.
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


def inyectar_paneles(mundo, array_paneles, modelo):
    """
    Lee un array de paneles e inyecta cada modelo en el mundo de Gazebo activo.
    
    Utiliza el comando de terminal 'gz service' para realizar el spawn del modelo
    con la posición y orientación (traducida a cuaterniones) especificadas.
    """
    # Ruta base al archivo SDF del modelo del panel
    base_dir = os.path.expanduser(
        "~/Carpeta_TFG_Provisional/src/TFG_inspeccion_plantas_termosolares"
    )
    ruta_modelo = os.path.join(
        base_dir, f"simulacion/simulacion/models/{modelo}.sdf"
    )
    
    for panel in array_paneles:
        # Extraemos los datos del diccionario
        id_panel = panel['id']
        x = panel['x']
        y = panel['y']
        z = panel['z']
        roll = panel.get('roll', 0.0)
        pitch = panel.get('pitch', 0.0)
        yaw = panel.get('yaw', 0.0)
        
        # Convertimos los ángulos para Gazebo 
        # (Roll siempre es 0 para un espejo apoyado en el suelo)
        qx, qy, qz, qw = euler_a_cuaternion(roll, pitch, yaw)
        
        # Comando de inyección mediante la CLI de Gazebo
        comando = (
            f"gz service -s /world/{mundo}/create "
            f"--reqtype gz.msgs.EntityFactory "
            f"--reptype gz.msgs.Boolean "
            f"--timeout 1000 "
            f"--req 'sdf_filename: \"{ruta_modelo}\", name: \"{id_panel}\", "
            f"pose: {{position: {{x: {x}, y: {y}, z: {z}}}, "
            f"orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}}}'"
        )
        
        # Ejecutamos silenciosamente en la terminal de Linux
        subprocess.run(
            comando, 
            shell=True, 
            stdout=subprocess.DEVNULL, 
            stderr=subprocess.DEVNULL
        )
        
