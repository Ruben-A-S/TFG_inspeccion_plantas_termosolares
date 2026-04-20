#!/usr/bin/env python3

import sys
import subprocess
import os

def spawn_panel(world, nombre, x, y, z, pitch, yaw):
    print(f"Python ordenando crear a: {nombre}...")
    
    # Preparamos la lista de comandos (como si lo escribiéramos en la terminal con espacios)
    
    directorio_actual = os.path.dirname(os.path.abspath(__file__))
    ruta_bash = os.path.join(directorio_actual, "add_panel.sh")
    
    comando = [
        ruta_bash, 
        str(world), 
        str(nombre), 
        str(x), 
        str(y), 
        str(z), 
        str(pitch), 
        str(yaw)
    ]
    
    # Ejecutamos el comando y esperamos a que termine
    resultado = subprocess.run(comando, capture_output=True, text=True)
    
    # Comprobamos si ha ido bien
    if resultado.returncode == 0:
        print(f" ¡{nombre} creado con éxito!")
        # Si quieres ver lo que respondió Bash, descomenta la siguiente línea:
        # print(resultado.stdout)
    else:
        print(f" Error al crear {nombre}:")
        print(resultado.stderr)
    
def inyectar_paneles(world, archivo_txt):
    """
    Lee un archivo de texto y genera los paneles en Gazebo.
    Devuelve True si termina con éxito, False si hay un error.
    """
    if not os.path.isfile(archivo_txt):
        print(f"[ERROR] No se encuentra el archivo de paneles: '{archivo_txt}'.")
        return False

    print(f"Leyendo mapa de obstáculos desde: {archivo_txt}\n")
    
    with open(archivo_txt, 'r') as file:
        lineas = file.readlines()

    for numero_linea, linea in enumerate(lineas, 1):
        linea_limpia = linea.strip()

        if not linea_limpia or linea_limpia.startswith('#'):
            continue

        parametros = linea_limpia.split()

        if len(parametros) != 6:
            print(f"Advertencia (Línea {numero_linea}): Se esperaban 6 parámetros, pero hay {len(parametros)}. Saltando línea...")
            continue

        nombre, x, y, z, pitch, yaw = parametros
        spawn_panel(world, nombre, x, y, z, pitch, yaw)
        
    print("[ÉXITO] Inyección de paneles finalizada.")
    return True

# --- PROTECCIÓN PARA USO EN TERMINAL ---
if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Error de sintaxis.")
        print("Uso correcto: python3 Add_panels_from_file.py <world_name> <ruta_archivo.txt>")
        sys.exit(1)
        
    world_arg = sys.argv[1]
    archivo_arg = sys.argv[2]   
    
    # Llamamos a nuestra nueva función
    inyectar_paneles(world_arg, archivo_arg)
