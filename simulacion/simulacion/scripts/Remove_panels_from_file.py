#!/usr/bin/env python3

import sys
import subprocess
import os

def kill_panel(world, nombre):
    print(f"Python ordenando eliminar a: {nombre}...")
    
    # TRUCO: Obtenemos la ruta absoluta de la carpeta donde está este script
    # Así siempre encontrará remove_panel.sh
    directorio_actual = os.path.dirname(os.path.abspath(__file__))
    ruta_bash = os.path.join(directorio_actual, "remove_panel.sh")

    # Preparamos la lista de comandos (Bash solo necesita el mundo y el nombre para borrar)
    comando = [
        ruta_bash, 
        str(world), 
        str(nombre)
    ]
    
    # Ejecutamos el comando y esperamos a que termine
    resultado = subprocess.run(comando, capture_output=True, text=True)
    
    # Comprobamos si ha ido bien
    if resultado.returncode == 0:
        print(f" ¡{nombre} eliminado con éxito!")
    else:
        print(f" Error al eliminar {nombre}:")
        print(resultado.stderr)
    
# --- NUEVA FUNCIÓN PARA IMPORTAR DESDE EL ORQUESTADOR ---
def eliminar_paneles(world, archivo_txt):
    """
    Lee un archivo de texto y elimina los paneles correspondientes en Gazebo.
    Devuelve True si termina con éxito, False si hay un error.
    """
    if not os.path.isfile(archivo_txt):
        print(f"[ERROR] No se encuentra el archivo: '{archivo_txt}'.")
        return False

    print(f"Leyendo mapa para eliminar obstáculos desde: {archivo_txt}\n")
    
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

        # Solo necesitamos extraer el nombre (la primera posición) para borrarlo
        nombre = parametros[0]
        
        kill_panel(world, nombre)
        
    print("[ÉXITO] Eliminación de paneles finalizada.")
    return True

# --- PROTECCIÓN PARA USO EN TERMINAL ---
if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Error de sintaxis.")
        print("Uso correcto: python3 Remove_panels_from_file.py <world_name> <archivo.txt>")
        sys.exit(1)
        
    world_arg = sys.argv[1]
    archivo_arg = sys.argv[2]   
    
    # Llamamos a nuestra nueva función
    eliminar_paneles(world_arg, archivo_arg)
