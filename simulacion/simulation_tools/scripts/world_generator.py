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
    # Plantilla XML/SDF con PBR integrado Y PLUGINS DE PX4
    contenido_sdf = f"""﻿<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <world name='{nombre_mundo}'>
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene>
      <grid>false</grid>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>1 1</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode/>
            </friction>
            <bounce/>
            <contact/>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>500 500</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <pbr>
              <metal>
                <albedo_map>{textura_absoluta}</albedo_map>
                <roughness>0.9</roughness> 
                <metalness>0.0</metalness> 
              </metal>
            </pbr>
          </material>
        </visual>
        <pose>0 0 0 0 -0 0</pose>
        <inertial>
          <pose>0 0 0 0 -0 0</pose>
          <mass>1</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
        <enable_wind>false</enable_wind>
      </link>
      <pose>0 0 0 0 -0 0</pose>
      <self_collide>false</self_collide>
    </model>
    <light name="sunUTC" type="directional">
      <pose>0 0 500 0 -0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1</intensity>
      <direction>0.001 0.625 -0.78</direction>
      <diffuse>0.904 0.904 0.904 1</diffuse>
      <specular>0.271 0.271 0.271 1</specular>
      <attenuation>
        <range>2000</range>
        <linear>0</linear>
        <constant>1</constant>
        <quadratic>0</quadratic>
      </attenuation>
      <spot>
        <inner_angle>0</inner_angle>
        <outer_angle>0</outer_angle>
        <falloff>0</falloff>
      </spot>
    </light>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>47.397971057728974</latitude_deg>
      <longitude_deg> 8.546163739800146</longitude_deg>
      <elevation>0</elevation>
    </spherical_coordinates>
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
