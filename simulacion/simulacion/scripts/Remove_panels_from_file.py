#!/usr/bin/env python3

import os
import subprocess
import sys


def _kill_panel(world, nombre):
    """
    Función interna que ejecuta un script bash para eliminar un modelo de Gazebo.
    """
    print(f"Python ordenando eliminar a: {nombre} en el mundo: {world}...")
    
    # Obtenemos la ruta absoluta del script .sh
    directorio_actual = os.path.dirname(os.path.abspath(__file__))
    ruta_bash = os.path.join(directorio_actual, "remove_panel.sh")

    # Preparamos el comando
    comando = [
        ruta_bash, 
        str(world), 
        str(nombre)
    ]
    
    # Ejecutamos capturando la salida para no ensuciar la terminal si no queremos
    resultado = subprocess.run(comando, capture_output=True, text=True)
    
    if resultado.returncode == 0:
        print(f"  [OK] ¡{nombre} eliminado con éxito!")
    else:
        print(f"  [ERROR] Error al eliminar {nombre}: {resultado.stderr}")


def eliminar_paneles(nombre_mundo, array_paneles):
    """
    Recorre una lista de paneles y los elimina uno a uno del entorno de Gazebo.
    
    :param nombre_mundo: Cadena con el nombre del mundo en Gazebo.
    :param array_paneles: Lista de diccionarios, donde cada panel tiene un 'id'.
    """
    for panel in array_paneles:
        # Extraemos el ID, que es lo único que Gazebo necesita para borrar
        id_panel = panel.get('id')
        
        if id_panel:
            _kill_panel(nombre_mundo, id_panel)
            
    print("[ÉXITO] Eliminación de paneles finalizada.")
    return True


if __name__ == "__main__":
    # --- PROTECCIÓN PARA USO EN TERMINAL INDEPENDIENTE ---
    if len(sys.argv) < 3:
        print("Uso: python3 panel_remover.py <mundo> <nombre_objeto>")
        sys.exit(1)
        
    world_arg = sys.argv[1]
    nombre_arg = sys.argv[2]   
    
    # Si lo usas por terminal, simulamos un array con un solo panel
    eliminar_paneles(world_arg, [{"id": nombre_arg}])
