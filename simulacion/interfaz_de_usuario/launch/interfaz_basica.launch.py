from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Define la ruta exacta donde tienes tus scripts de Python
    # Ajusta esto si en el futuro mueves la carpeta
    ruta_scripts = '/home/ruben/Carpeta_TFG_Provisional/src/TFG_inspeccion_plantas_termosolares/simulacion/interfaz_de_usuario/scripts'
    
    return LaunchDescription([
        # 1. NODO DE INTERFAZ BASICA
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'bash', '-c', 'python3 interfaz_terminal_node.py'],
            cwd=ruta_scripts,
            output='screen'
        ),
        
        # 2. RViz 2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', ruta_scripts + '/rviz.rviz']
        )
    ])
