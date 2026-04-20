from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Define la ruta exacta donde tienes tus scripts de Python
    # Ajusta esto si en el futuro mueves la carpeta
    ruta_scripts = '/home/ruben/Carpeta_TFG_Provisional/src/TFG_inspeccion_plantas_termosolares/simulacion/simulacion/scripts'
    
    return LaunchDescription([
        # 1. NODO DE CONTROL DE SIMULACION
        ExecuteProcess(
            # OJO: Separado por comas ('python3', 'nombre_del_script.py')
            cmd=['python3', 'control_sim_node.py'], 
            cwd=ruta_scripts,
            output='screen'
        ),
        
        # 2. NODO DE CARGA DEL MAPA (Comentado correctamente con #)
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen',
        #     arguments=['-d', ruta_scripts + '/rviz.rviz']
        # )
    ])
