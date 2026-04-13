import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import threading
import time

class SimCliNode(Node):
    """Nodo publicador puro para enviar comandos desde la terminal"""
    def __init__(self):
        super().__init__('sim_cli_node')
        self.pub_mundo = self.create_publisher(String, '/sim_cmd/config_mundo', 10)
        self.pub_paneles = self.create_publisher(String, '/sim_cmd/config_paneles', 10)
        self.pub_dron = self.create_publisher(String, '/sim_cmd/config_dron', 10)
        self.pub_accion = self.create_publisher(String, '/sim_cmd/accion', 10)

    def publicar_json(self, publicador, diccionario):
        msg = String()
        # json.dumps() convierte automáticamente el diccionario en un texto con comillas dobles perfectas
        msg.data = json.dumps(diccionario)
        publicador.publish(msg)

    def publicar_accion(self, accion):
        msg = String()
        msg.data = accion
        self.pub_accion.publish(msg)


def menu_interactivo(nodo):
    """Bucle infinito que lee del teclado usando input()"""
    time.sleep(0.5) # Pequeña pausa para asegurar que ROS 2 conecta
    
    while True:
        print("\n" + "="*45)
        print("   PANEL DE CONTROL DE SIMULACIÓN ")
        print("="*45)
        print("1.  Configurar Mundo (Nombre y Textura)")
        print("2.  Configurar Paneles (Archivo CSV)")
        print("3.  Configurar Dron (Modelo y Posición)")
        print("-"*45)
        print("4.   LANZAR SIMULACIÓN (GENERAR)")
        print("5.   Detener Simulación (TERMINAR)")
        print("6.  Apagar Todo y Salir (SALIR)")
        print("="*45)

        # LA LECTURA POR TECLADO
        opcion = input(" Elige una opción (1-6): ")

        if opcion == '1':
            # Si el usuario solo pulsa Enter, coge el valor después del 'or'
            nombre = input("   Nombre del mundo [prueba1]: ") or "prueba1"
            textura = input("   Ruta de textura [none]: ") or "none"
            nodo.publicar_json(nodo.pub_mundo, {"nombre": nombre, "textura": textura})
            print("   [OK] Datos del mundo enviados al Orquestador.")

        elif opcion == '2':
            ruta = input("   Ruta del CSV [mapa_3.txt]: ") or "mapa_3.txt"
            modelo = input("   Modelo del panel [panel]: ") or "panel"
            nodo.publicar_json(nodo.pub_paneles, {"ruta_csv": ruta, "modelo": modelo})
            print("   [OK] Datos de paneles enviados al Orquestador.")

        elif opcion == '3':
            modelo = input("   Modelo de dron [x500]: ") or "x500"
            try:
                x = float(input("   Coordenada X (ej. 5.0) [0.0]: ") or "0.0")
                y = float(input("   Coordenada Y (ej. -2.0) [0.0]: ") or "0.0")
                nodo.publicar_json(nodo.pub_dron, {"modelo": modelo, "x": x, "y": y})
                print("   [OK] Datos del dron enviados al Orquestador.")
            except ValueError:
                print("   [ERROR] Las coordenadas deben ser números. Inténtelo de nuevo.")

        elif opcion == '4':
            print("\n   >>  Enviando orden de GENERAR...")
            nodo.publicar_accion("GENERAR")

        elif opcion == '5':
            print("\n   >>  Enviando orden de TERMINAR...")
            nodo.publicar_accion("TERMINAR")

        elif opcion == '6':
            print("\n   >>  Apagando el Orquestador y saliendo...")
            nodo.publicar_accion("SALIR")
            break # Rompe el bucle while y termina este script
        
        else:
            print("   [!] Opción no válida. Escribe un número del 1 al 6.")


def main():
    rclpy.init()
    nodo = SimCliNode()

    # TRUCO DE HILOS: 
    # ROS 2 gira en segundo plano para poder enviar mensajes
    hilo_ros = threading.Thread(target=rclpy.spin, args=(nodo,), daemon=True)
    hilo_ros.start()

    # El hilo principal se queda atrapado en los input() esperando al usuario
    try:
        menu_interactivo(nodo)
    except KeyboardInterrupt:
        print("\nSaliendo del panel por teclado (Ctrl+C).")

    nodo.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()
