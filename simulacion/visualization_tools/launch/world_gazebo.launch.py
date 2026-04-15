import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, AppendEnvironmentVariable
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. Encontrar la ruta de nuestro paquete compilado
    pkg_share = FindPackageShare('simulation_tools').find('simulation_tools')
    
    # 2. ¡EL CONJURO DE LA VISIÓN! 
    # Le decimos a Gazebo dónde están físicamente nuestros modelos
    models_path = os.path.join(pkg_share, 'models')
    set_model_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        models_path
    )
    
    # 3. Definir la ruta exacta a su mundo (Veo que está usando el de la cancha)
    world_file = os.path.join(pkg_share, 'worlds', 'prueba1.sdf')
    
    # 4. Crear el comando para lanzar Gazebo
    gazebo_process = ExecuteProcess(
        cmd=['gz', 'sim', world_file, '-r'],
        output='screen'
    )

    # 5. Empaquetar y devolver la orden (¡Añadiendo la variable de entorno primero!)
    return LaunchDescription([
        set_model_path,
        gazebo_process
    ])
