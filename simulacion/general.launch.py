from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    config_path = '/home/ruben/Carpeta_TFG_Provisional/src/TFG_inspeccion_plantas_termosolares/simulacion/config'
    
    ui_scripts_path = '/home/ruben/Carpeta_TFG_Provisional/src/TFG_inspeccion_plantas_termosolares/simulacion/interfaz_de_usuario/scripts'
    
    simulation_scripts_path = '/home/ruben/Carpeta_TFG_Provisional/src/TFG_inspeccion_plantas_termosolares/simulacion/simulacion/scripts'
    
    faker_scripts_path = '/home/ruben/Carpeta_TFG_Provisional/src/TFG_inspeccion_plantas_termosolares/simulacion/faker_procesado/scripts'
    
    calculation_scripts_path = '/home/ruben/Carpeta_TFG_Provisional/src/TFG_inspeccion_plantas_termosolares/simulacion/metodo_de_calculo/scripts'
    
    return LaunchDescription([
        # 1. Launch interface
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'bash', '-c', 'python3 terminal_interface_node.py; exec bash'],
            cwd=ui_scripts_path,
            output='screen'
        ),
        
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'bash', '-c', 'python3 feedback_monitor_node.py; exec bash'],
            cwd=ui_scripts_path,
            output='screen'
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', config_path + '/rviz.rviz']
        ),
        
        # 2. Launch simulation
        ExecuteProcess(
            cmd=['python3', 'sim_orchestrator_node.py'], 
            cwd=simulation_scripts_path,
            output='screen'
        ),
        ExecuteProcess(
            cmd=['python3', 'map_loader_node.py'], 
            cwd=simulation_scripts_path,
            output='screen'
        ),
        
        # 3. Launch faker
        ExecuteProcess(
            cmd=['python3', 'optics_calculator_node.py'],
            cwd=faker_scripts_path,
            output='screen'
        ),
        
        ExecuteProcess(
            cmd=['python3', 'rviz_visualizer_node.py'],
            cwd=faker_scripts_path,
            output='screen'
        ),
        
        ExecuteProcess(
            cmd=['python3', 'virtual_camera_node.py'],
            cwd=faker_scripts_path,
            output='screen'
        ),
        
        ExecuteProcess(
            cmd=['python3', 'camera_filter_node.py'],
            cwd=faker_scripts_path,
            output='screen'
        ),
        
        # 4. Launch calculation
        ExecuteProcess(
            cmd=['python3', 'calibration_node.py'],
            cwd=calculation_scripts_path,
            output='screen'
        ),
        
        ExecuteProcess(
            cmd=['python3', 'rviz_calibration_markers_node.py'],
            cwd=calculation_scripts_path,
            output='screen'
        ),
            
        ExecuteProcess(
            cmd=['python3', 'collector_analysis_logger_node.py'],
            cwd=calculation_scripts_path,
            output='screen'
        )            
    ])
