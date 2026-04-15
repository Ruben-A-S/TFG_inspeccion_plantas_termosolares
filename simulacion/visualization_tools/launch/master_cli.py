#!/usr/bin/env python3
import os
import subprocess
import sys
import time

def main():
    # 1. Definir rutas absolutas (ajuste si es necesario)
    dir_actual = os.path.dirname(os.path.abspath(__file__))
    base_path = os.path.abspath(os.path.join(dir_actual, ".."))
    ruta_generador = os.path.join(dir_actual, "..", "scripts/world_generator.py")
    ruta_add_file = os.path.join(dir_actual, "..", "scripts/Add_panels_from_file.py")
    ruta_rm_file = os.path.join(dir_actual, "..", "scripts/Remove_panels_from_file.py")
    ruta_add_sh = os.path.join(dir_actual, "..", "scripts/add_panel.sh")
    ruta_rm_sh = os.path.join(dir_actual, "..", "scripts/remove_panel.sh")

    print("="*50)
    print(" INICIANDO SISTEMA ORQUESTADOR DEL TFG ")
    print("="*50)

    # --- FASE 1: GENERACIÓN DEL MUNDO ---
    print("\n[FASE 1] Configuración del Entorno")
    world_name = input(" -> Ingrese el nombre del mundo (ej. prueba1): ").strip()
    
    print(" -> Ingrese la ruta de la textura del suelo")
    print("    (Deje en blanco para usar basketball_court.png por defecto):")
    textura = input(" -> ").strip()
    if not textura:
        textura = os.path.join(base_path, "models", "textures", "basketball_court.png")

    ruta_salida = os.path.join(base_path, "worlds", f"{world_name}.sdf")

    print("\nGenerando archivo .sdf...")
    subprocess.run(["python3", ruta_generador, "--nombre", world_name, "--textura", textura, "--salida", ruta_salida])


    # --- FASE 2: LANZAR GAZEBO EN SEGUNDO PLANO ---
    print("\n[FASE 2] Arrancando Gazebo...")
    # Usamos Popen para que no bloquee el script. 
    # Redirigimos la salida a un archivo (gazebo_log.txt) para que no ensucie nuestro menú de la terminal
    log_file = open("gazebo_log.txt", "w")
    cmd_gazebo = "source ~/Carpeta_TFG_Provisional/install/setup.bash && ros2 launch simulation_tools world_gazebo.launch.py"
    
    proceso_gazebo = subprocess.Popen(
        cmd_gazebo, 
        shell=True, 
        executable="/bin/bash",
        stdout=log_file,
        stderr=subprocess.STDOUT
    )
    
    print("Esperando 5 segundos a que Gazebo despierte...")
    time.sleep(5)


    # --- FASE 3: BUCLE INTERACTIVO (MENU CLI) ---
    while True:
        print("\n" + "="*40)
        print(" PANEL DE CONTROL DE PANELES SOLARES")
        print("="*40)
        print(" 1. Añadir panel INDIVIDUAL")
        print(" 2. Eliminar panel INDIVIDUAL")
        print(" 3. Añadir paneles desde ARCHIVO (.txt/.csv)")
        print(" 4. Eliminar paneles desde ARCHIVO")
        print(" 0. Salir y cerrar Gazebo")
        print("="*40)
        
        opcion = input("Seleccione una opción >> ").strip()

        if opcion == '1':
            print("\n-- Añadir Panel Individual --")
            nombre = input("Nombre del panel: ")
            x = input("Posición X: ")
            y = input("Posición Y: ")
            z = input("Posición Z: ")
            pitch = input("Inclinación (Pitch en radianes): ")
            yaw = input("Orientación (Yaw en radianes): ")
            subprocess.run([ruta_add_sh, world_name, nombre, x, y, z, pitch, yaw])

        elif opcion == '2':
            print("\n-- Eliminar Panel Individual --")
            nombre = input("Nombre del panel a destruir: ")
            subprocess.run([ruta_rm_sh, world_name, nombre])

        elif opcion == '3':
            print("\n-- Añadir desde Archivo --")
            print("Ejemplo: /home/ruben/TFG/workspace_docker/Add-Espejos/mapa_1.txt")
            ruta_txt = input("Ruta absoluta del archivo: ").strip()
            subprocess.run(["python3", ruta_add_file, world_name, ruta_txt])

        elif opcion == '4':
            print("\n-- Eliminar desde Archivo --")
            ruta_txt = input("Ruta absoluta del archivo: ").strip()
            subprocess.run(["python3", ruta_rm_file, world_name, ruta_txt])

        elif opcion == '0':
            print("\nIniciando secuencia de apagado...")
            # Matar el proceso de Gazebo
            proceso_gazebo.terminate()
            log_file.close()
            print("¡Hasta la próxima, ingeniero!")
            sys.exit(0)

        else:
            print("Opción no válida. Inténtelo de nuevo.")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrupción detectada. Cerrando de emergencia...")
        sys.exit(0)
