import subprocess
import math
import os

def euler_a_cuaternion(roll, pitch, yaw):
    """Convierte ángulos de Euler a Cuaterniones (x, y, z, w) para Gazebo"""
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return qx, qy, qz, qw

def inyectar_paneles(mundo, array_paneles, modelo):
    # Asegúrate de que esta ruta base apunta a donde tienes guardado el modelo del panel
    ruta_modelo = os.path.expanduser(f"~/Carpeta_TFG_Provisional/src/TFG_inspeccion_plantas_termosolares/simulacion/simulacion/models/{modelo}.sdf")
    
    for panel in array_paneles:
        # Extraemos los datos del diccionario
        id_panel = panel['id']
        x = panel['x']
        y = panel['y']
        z = panel['z']
        pitch = panel['pitch']
        yaw = panel['yaw']
        
        # Convertimos los ángulos para Gazebo (Roll siempre es 0 para un espejo plantado en el suelo)
        qx, qy, qz, qw = euler_a_cuaternion(0.0, pitch, yaw)
        
        # Comando de inyección (Fíjate que aquí usamos la variable {mundo})
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
        subprocess.run(comando, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
