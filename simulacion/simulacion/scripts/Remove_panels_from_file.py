#!/usr/bin/env python3

import sys
import subprocess
import os

def kill_panel(world, nombre):
    print(f"Python ordenando eliminar a: {nombre} en el mundo: {world}...")
    
    # Obtenemos la ruta del script .sh
    directorio_actual = os.path.dirname(os.path.abspath(__file__))
    ruta_bash = os.path.join(directorio_actual, "remove_panel.sh")

    # Preparamos el comando
    comando = [
        ruta_bash, 
        str(world), 
        str(nombre)
    ]
    
    # Ejecutamos
    resultado = subprocess.run(comando, capture_output=True, text=True)
    
    if resultado.returncode == 0:
        print(f"  ¡{nombre} eliminado con éxito!")
    else:
        print(f"  Error al eliminar {nombre}: {resultado.stderr}")
    
# --- FUNCIÓN CORREGIDA ---
def eliminar_paneles(nombre_mundo, array_paneles):
    """
    nombre_mundo: string con el nombre del mundo en Gazebo
    array_paneles: lista de diccionarios (cada uno con la clave 'id')
    """
    for panel in array_paneles:
        # Extraemos el ID que es lo único que Gazebo necesita para borrar
        id_panel = panel.get('id')
        
        if id_panel:
            # CORRECCIÓN: Usamos las variables correctas que recibe la función
            kill_panel(nombre_mundo, id_panel)
        
    print("[ÉXITO] Eliminación de paneles finalizada.")
    return True

# --- PROTECCIÓN PARA USO EN TERMINAL (Si decides usarlo a mano) ---
if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Uso: python3 Remove_panels_from_file.py <mundo> <nombre_objeto>")
        sys.exit(1)
        
    world_arg = sys.argv[1]
    nombre_arg = sys.argv[2]   
    
    # Si lo usas por terminal, simulamos un array con un solo panel
    eliminar_paneles(world_arg, [{"id": nombre_arg}])
