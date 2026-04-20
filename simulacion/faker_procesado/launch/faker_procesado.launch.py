from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Define la ruta exacta donde tienes tus scripts de Python
    # Ajusta esto si en el futuro mueves la carpeta
    ruta_scripts = '/home/ruben/Carpeta_TFG_Provisional/src/TFG_inspeccion_plantas_termosolares/simulacion/faker_procesado/scripts'
    
    return LaunchDescription([
        # 1. NODO DE CALCULO
        ExecuteProcess(
            cmd=['python3 calcule_node.py'],
            cwd=ruta_scripts,
            output='screen'
        ),
        
        # 2. NODO DE PUBLICACION DE MARCADORES BASICOS
        ExecuteProcess(
            cmd=['python3 visualize_node.py'],
            cwd=ruta_scripts,
            output='screen'
        ),
        
        # 3. NODO DE PUBLICACION DE MARCADORES Y IMAGEN TEORICA DE CAMARA
        ExecuteProcess(
            cmd=['python3 area_camera_node.py'],
            cwd=ruta_scripts,
            output='screen'
        ),
        
        # 3. NODO DE PUBLICACION DE DATOS DE PROCESADO TEORICO
        ExecuteProcess(
            cmd=['python3 show_data_node.py'],
            cwd=ruta_scripts,
            output='screen'
        )
    ])
