import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    base_dir = os.path.dirname(os.path.abspath(__file__))
    
    config_path = os.path.join(base_dir, 'config')
    ui_scripts_path = os.path.join(base_dir, 'interfaz_de_usuario', 'scripts')
    simulation_scripts_path = os.path.join(base_dir, 'simulacion', 'scripts')
    faker_scripts_path = os.path.join(base_dir, 'faker_procesado', 'scripts')
    calculation_scripts_path = os.path.join(base_dir, 'metodo_de_calculo', 'scripts')
    
    parameters_yaml_file = os.path.join(config_path, 'parameters.yaml')
    rviz_config_file = os.path.join(config_path, 'rviz.rviz')
    
    param_args = f"--ros-args --params-file {parameters_yaml_file}"

    return LaunchDescription([
        # 1. Launch interface
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'bash', '-c', f'python3 terminal_interface_node.py {param_args}; exec bash'],
            cwd=ui_scripts_path,
            output='screen'
        ),
        
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'bash', '-c', f'python3 feedback_monitor_node.py {param_args}; exec bash'],
            cwd=ui_scripts_path,
            output='screen'
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        ),
        
        # 2. Launch simulation
        ExecuteProcess(
            cmd=['python3', 'sim_orchestrator_node.py', '--ros-args', '--params-file', parameters_yaml_file], 
            cwd=simulation_scripts_path,
            output='screen'
        ),
        ExecuteProcess(
            cmd=['python3', 'map_loader_node.py', '--ros-args', '--params-file', parameters_yaml_file], 
            cwd=simulation_scripts_path,
            output='screen'
        ),
        
        # 3. Launch faker
        ExecuteProcess(
            cmd=['python3', 'optics_calculator_node.py', '--ros-args', '--params-file', parameters_yaml_file],
            cwd=faker_scripts_path,
            output='screen'
        ),
        
        ExecuteProcess(
            cmd=['python3', 'rviz_visualizer_node.py', '--ros-args', '--params-file', parameters_yaml_file],
            cwd=faker_scripts_path,
            output='screen'
        ),
        
        ExecuteProcess(
            cmd=['python3', 'virtual_camera_node.py', '--ros-args', '--params-file', parameters_yaml_file],
            cwd=faker_scripts_path,
            output='screen'
        ),
        
        ExecuteProcess(
            cmd=['python3', 'camera_filter_node.py', '--ros-args', '--params-file', parameters_yaml_file],
            cwd=faker_scripts_path,
            output='screen'
        ),
        
        # 4. Launch calculation
        ExecuteProcess(
            cmd=['python3', 'calibration_node.py', '--ros-args', '--params-file', parameters_yaml_file],
            cwd=calculation_scripts_path,
            output='screen'
        ),
        
        ExecuteProcess(
            cmd=['python3', 'rviz_calibration_markers_node.py', '--ros-args', '--params-file', parameters_yaml_file],
            cwd=calculation_scripts_path,
            output='screen'
        ),
            
        ExecuteProcess(
            cmd=['python3', 'collector_analysis_logger_node.py', '--ros-args', '--params-file', parameters_yaml_file],
            cwd=calculation_scripts_path,
            output='screen'
        )            
    ])
