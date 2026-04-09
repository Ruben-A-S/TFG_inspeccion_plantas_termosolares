import sys
import subprocess
import os

def kill_panel(world, nombre, x, y, z, pitch, yaw):
    print(f"Python ordenando eliminar a: {nombre}...")
    
    # Preparamos la lista de comandos (como si lo escribiéramos en la terminal con espacios)
    comando = [
        "./remove_panel.sh", 
        str(world), 
        str(nombre), 
    ]
    
    # Ejecutamos el comando y esperamos a que termine
    resultado = subprocess.run(comando, capture_output=True, text=True)
    
    # Comprobamos si ha ido bien
    if resultado.returncode == 0:
        print(f" ¡{nombre} eliminado con éxito!")
        # Si quieres ver lo que respondió Bash, descomenta la siguiente línea:
        # print(resultado.stdout)
    else:
        print(f" Error al eliminar {nombre}:")
        print(resultado.stderr)
    
def main():
    # 1. Comprobamos que se le ha pasado un archivo al llamar al script
    if len(sys.argv) != 3:
        print("Error de sintaxis.")
        print("Uso correcto: python3 Remove_panels_from_file.py <world_name> <archivo.txt>")
        sys.exit(1)
        
    world = sys.argv[1]
    archivo_txt = sys.argv[2]    
    
    # 2. Comprobamos que el archivo existe realmente
    if not os.path.isfile(archivo_txt):
        print(f"Error: No se encuentra el archivo '{archivo_txt}'.")
        sys.exit(1)

    print(f"Leyendo mapa de obstáculos desde: {archivo_txt}\n")
    
    # 3. Abrimos el archivo y lo leemos línea a línea
    with open(archivo_txt, 'r') as file:
        lineas = file.readlines()

    for numero_linea, linea in enumerate(lineas, 1):
        # Limpiamos espacios extra y saltos de línea
        linea_limpia = linea.strip()

        # Ignoramos las líneas vacías y los comentarios (líneas que empiezan por #)
        if not linea_limpia or linea_limpia.startswith('#'):
            continue

        # Separamos la línea por espacios. Asumimos el orden: nombre x y z pitch yaw
        parametros = linea_limpia.split()

        # Comprobamos que la línea tiene exactamente los 6 datos que necesita Bash
        if len(parametros) != 6:
            print(f"Advertencia (Línea {numero_linea}): Se esperaban 6 parámetros, pero hay {len(parametros)}. Saltando línea...")
            continue

        # Desempaquetamos las variables
        nombre, x, y, z, pitch, yaw = parametros
    
        kill_panel(world, nombre, x, y, z, pitch, yaw);
            
if __name__ == "__main__":
    main();
    
    
    

