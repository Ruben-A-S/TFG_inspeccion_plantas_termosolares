from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Define la ruta exacta donde tienes tus scripts de Python
    # Ajusta esto si en el futuro mueves la carpeta
    ruta_scripts = '/home/ruben/Carpeta_TFG_Provisional/src/TFG_inspeccion_plantas_termosolares/simulacion/visualization_tools/scripts'

    # 1. DECLARACIÓN DE ARGUMENTOS (Con valores por defecto para que funcione sin teclear nada)
    arg_mundo = DeclareLaunchArgument(
        'mundo',
        default_value='prueba1',
        description='Nombre del mundo en Gazebo'
    )
    
    arg_mapa = DeclareLaunchArgument(
        'mapa',
        default_value='mapa_3.txt',
        description='Archivo de texto con el mapa de paneles'
    )
    
    arg_dron = DeclareLaunchArgument(
        'dron',
        default_value='x500',
        description='Nombre exacto del modelo del dron que escupe Gazebo'
    )
    
    return LaunchDescription([
        arg_mundo,
        arg_mapa,
        arg_dron,
        
        # 2. NODO DE CALCULO (con los argumentos que averiguamos antes)
        ExecuteProcess(
            cmd=['python3', 'calcule_node.py', LaunchConfiguration('mundo'), LaunchConfiguration('mapa'), LaunchConfiguration('dron')],
            cwd=ruta_scripts,
            output='screen'
        ),
        
        # 3. NODO DE VISUALIZACION BASICO (he asumido que se llama visualizador_node.py según el código anterior)
        ExecuteProcess(
            cmd=['python3', 'visualize_node.py'],
            cwd=ruta_scripts,
            output='screen'
        ),

        # 4. NODO DE VISUALIZACION CON MODELO CAMARA
        ExecuteProcess(
            cmd=['python3', 'area_camara_node_2.py'],
            cwd=ruta_scripts,
            output='screen'
        ),

        # 5. RViz 2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', ruta_scripts + '/rviz.rviz']
        )
    ])
