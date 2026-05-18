from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    ruta_config = '/home/ruben/Carpeta_TFG_Provisional/src/TFG_inspeccion_plantas_termosolares/simulacion/config'
    
    ruta_scripts_interfaz = '/home/ruben/Carpeta_TFG_Provisional/src/TFG_inspeccion_plantas_termosolares/simulacion/interfaz_de_usuario/scripts'
    
    ruta_scripts_simulacion = '/home/ruben/Carpeta_TFG_Provisional/src/TFG_inspeccion_plantas_termosolares/simulacion/simulacion/scripts'
    
    ruta_scripts_faker = '/home/ruben/Carpeta_TFG_Provisional/src/TFG_inspeccion_plantas_termosolares/simulacion/faker_procesado/scripts'
    
    ruta_scripts_calculo = '/home/ruben/Carpeta_TFG_Provisional/src/TFG_inspeccion_plantas_termosolares/simulacion/metodo_de_calculo/scripts'
    
    return LaunchDescription([
        # 1. Launch interfaz
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'bash', '-c', 'python3 interfaz_terminal_node.py; exec bash'],
            cwd=ruta_scripts_interfaz,
            output='screen'
        ),
        
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'bash', '-c', 'python3 feedback_monitor_node.py; exec bash'],
            cwd=ruta_scripts_interfaz,
            output='screen'
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', ruta_config + '/rviz.rviz']
        ),
        
        # 2. Launch simulacion
        ExecuteProcess(
            cmd=['python3', 'sim_orchestrator_node.py'], 
            cwd=ruta_scripts_simulacion,
            output='screen'
        ),
        ExecuteProcess(
            cmd=['python3', 'map_loader_node.py'], 
            cwd=ruta_scripts_simulacion,
            output='screen'
        ),
        
        # 3. Launch faker
        ExecuteProcess(
            cmd=['python3', 'optics_calculator_node.py'],
            cwd=ruta_scripts_faker,
            output='screen'
        ),
        
        ExecuteProcess(
            cmd=['python3', 'rviz_visualizer_node.py'],
            cwd=ruta_scripts_faker,
            output='screen'
        ),
        
        ExecuteProcess(
            cmd=['python3', 'virtual_camera_node.py'],
            cwd=ruta_scripts_faker,
            output='screen'
        ),
        
        ExecuteProcess(
            cmd=['python3', 'camera_filter_node.py'],
            cwd=ruta_scripts_faker,
            output='screen'
        ),
        
        ExecuteProcess(
            cmd=['python3', 'heliopoint_calibration_node.py'],
            cwd=ruta_scripts_calculo,
            output='screen'
        ),
        
        ExecuteProcess(
            cmd=['python3', 'rviz_helio_markers_node.py'],
            cwd=ruta_scripts_calculo,
            output='screen'
        )                
    ])
