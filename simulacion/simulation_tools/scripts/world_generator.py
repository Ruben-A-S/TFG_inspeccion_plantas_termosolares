import argparse
import os

def crear_mundo_base(nombre_mundo, ruta_textura, ruta_salida):
    """
    Genera un archivo .sdf con un sol, físicas estándar y un plano con textura PBR.
    """
    # Verificamos si la textura existe para avisar al usuario
    if not os.path.exists(ruta_textura):
        print(f"[ADVERTENCIA] No se ha encontrado la imagen en: {ruta_textura}")
        print("Asegúrese de que la ruta sea correcta o Gazebo pintará el suelo de blanco/negro.")

    # Convertimos la ruta de la imagen a ruta absoluta para que Gazebo siempre la encuentre
    textura_absoluta = os.path.abspath(ruta_textura)

    # Plantilla XML/SDF con PBR integrado
    contenido_sdf = f"""<?xml version="1.0" ?>
<sdf version='1.6'>
  <world name='{nombre_mundo}'>
    <physics name='default_physics' default='0' type='ode'>
      <max_step_size>0.002</max_step_size>
      <real_time_update_rate>500</real_time_update_rate>
    </physics>

    <light name='sun' type='directional'>
      <cast_shadows>1</cast_shadows>
      <pose>0 0 10 0 -0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name='suelo_custom'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <box>
              <size>200 200 0.1</size> </box>
          </geometry>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>200 200 0.1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>1 1 1 1</diffuse>
            <pbr>
              <metal>
                <albedo_map>{textura_absoluta}</albedo_map>
                <roughness>0.9</roughness> <metalness>0.0</metalness> </metal>
            </pbr>
          </material>
        </visual>
      </link>
    </model>
    
    </world>
</sdf>
"""
    # Crear el directorio de salida si no existe
    os.makedirs(os.path.dirname(ruta_salida), exist_ok=True)

    # Escribir el archivo final
    with open(ruta_salida, 'w') as f:
        f.write(contenido_sdf)
        
    print(f"[ÉXITO] Mundo '{nombre_mundo}' generado correctamente en: {ruta_salida}")


if __name__ == "__main__":
    # Configuración de los argumentos por terminal
    parser = argparse.ArgumentParser(description="Generador de mundos base para Gazebo")
    
    parser.add_argument("--nombre", type=str, required=True, 
                        help="Nombre del mundo (ej. planta_sevilla)")
    
    parser.add_argument("--textura", type=str, required=True, 
                        help="Ruta a la imagen jpg/png para el suelo")
                        
    parser.add_argument("--salida", type=str, required=True, 
                        help="Ruta y nombre del archivo .sdf a generar")
    
    args = parser.parse_args()
    
    # Ejecutar la función principal
    crear_mundo_base(args.nombre, args.textura, args.salida)
