#!/usr/bin/env python3
"""
Base world generator script for Gazebo.
Creates an .sdf file with a sun, standard physics, and a plane with PBR texture.
"""

import argparse
import os


def create_base_world(world_name: str, texture_path: str, output_path: str, 
                      lat: float, lon: float, elevation: float) -> None:
    """
    Generates a Gazebo .sdf file with basic configuration and a ground texture.
    
    :param world_name: Internal name of the world in Gazebo.
    :param texture_path: Path to the image (jpg/png) that will be used as albedo_map.
    :param output_path: Full path where the .sdf file will be saved.
    :param lat: Latitude for the spherical coordinates.
    :param lon: Longitude for the spherical coordinates.
    :param elevation: Elevation (Altitude) over sea level.
    """
    # We verify if the texture exists to warn the user
    if not os.path.exists(texture_path):
        print(f"[WARNING] Image not found at: {texture_path}")
        print("Ensure the path is correct or Gazebo will paint the ground black/white.")

    # We convert the image path to an absolute path for Gazebo
    absolute_texture = os.path.abspath(texture_path)

    # XML/SDF template with integrated PBR and base plugins.
    # (In structured multiline text templates, PEP 8's 79 character limit 
    # is ignored so as not to corrupt the XML format).
    sdf_content = f"""<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <world name='{world_name}'>
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
                <albedo_map>{absolute_texture}</albedo_map>
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
      <latitude_deg>{lat}</latitude_deg>
      <longitude_deg>{lon}</longitude_deg>
      <elevation>{elevation}</elevation>
    </spherical_coordinates>
  </world>
</sdf>
"""
    
    # Create the output directory if it doesn't exist, preventing errors if only a file is passed
    output_directory = os.path.dirname(output_path)
    if output_directory:
        os.makedirs(output_directory, exist_ok=True)

    # Write the final file
    with open(output_path, 'w', encoding='utf-8') as f:
        f.write(sdf_content)
        
    print(f"[SUCCESS] World '{world_name}' successfully generated at: {output_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Base world generator for Gazebo")
    
    parser.add_argument("--name", type=str, required=True, help="Name of the world")
    parser.add_argument("--texture", type=str, required=True, help="Path to the texture")
    parser.add_argument("--output", type=str, required=True, help="Path to the output .sdf")
    
    # Nuevos argumentos para ejecución por terminal
    parser.add_argument("--lat", type=float, default=37.0934, help="Latitude")
    parser.add_argument("--lon", type=float, default=-6.7337, help="Longitude")
    parser.add_argument("--elevation", type=float, default=349.0, help="Elevation")
    
    args = parser.parse_args()
    create_base_world(args.name, args.texture, args.output, args.lat, args.lon, args.elevation)
