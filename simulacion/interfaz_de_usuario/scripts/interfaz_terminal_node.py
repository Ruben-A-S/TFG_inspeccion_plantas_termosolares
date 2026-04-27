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
        self.pub_fecha = self.create_publisher(String, '/sim_cmd/config_fecha', 10)
        self.pub_paneles = self.create_publisher(String, '/sim_cmd/config_paneles', 10)
        self.pub_dron = self.create_publisher(String, '/sim_cmd/config_dron', 10)
        self.pub_accion = self.create_publisher(String, '/sim_cmd/accion', 10)
        self.pub_camara = self.create_publisher(String, '/sim_cmd/config_camara', 10)

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
        print("2.  Configurar Fecha y Hora")
        print("3.  Configurar Paneles (Archivo CSV y Modelo)")
        print("4.  Configurar Dron (Modelo y Posición)")
        print("-" * 45)
        print("5.  Girar Camara en vivo (Grados)")
        print("-" * 45)        
        print("6.  Poblar mundo")
        print("7.  Vaciar mundo")
        print("-" * 45)
        print("8.  LANZAR SIMULACIÓN (GENERAR)")
        print("9.  Detener Simulación (TERMINAR)")
        print("10.  Apagar Todo y Salir (SALIR)")
        print("="*45)

        # LA LECTURA POR TECLADO
        opcion = input(" Elige una opción (1-10): ")

        if opcion == '1':
            # Si el usuario solo pulsa Enter, coge el valor después del 'or'
            nombre = input("   Nombre del mundo [prueba1]: ") or "prueba1"
            textura = input("   Ruta de textura [none]: ") or "none"
            nodo.publicar_json(nodo.pub_mundo, {"nombre": nombre, "textura": textura})
            print("   [OK] Datos del mundo enviados al Orquestador.")

        elif opcion == '2':
            # NUEVO BLOQUE: Lectura de Fecha y Hora
            fecha = input("   Fecha (ej. 10/02/2001) [10/02/2001]: ") or "10/02/2001"
            hora = input("   Hora (ej. 12:34) [12:34]: ") or "12:34"
            nodo.publicar_json(nodo.pub_fecha, {"fecha": fecha, "hora": hora})
            print("   [OK] Datos de fecha y hora enviados al Orquestador.")

        elif opcion == '3':
            ruta = input("   Ruta del CSV [mapa_3.txt]: ") or "Crescent_Dunes.csv"
            modelo = input("   Modelo del panel [panel]: ") or "panel"
            nodo.publicar_json(nodo.pub_paneles, {"ruta_csv": ruta, "modelo": modelo})
            print("   [OK] Datos de paneles enviados al Orquestador.")

        elif opcion == '4':
            modelo = input("   Modelo de dron [x500]: ") or "x500"
            try:
                x = float(input("   Coordenada X (ej. 5.0) [0.0]: ") or "0.0")
                y = float(input("   Coordenada Y (ej. -2.0) [0.0]: ") or "0.0")
                nodo.publicar_json(nodo.pub_dron, {"modelo": modelo, "x": x, "y": y})
                print("   [OK] Datos del dron enviados al Orquestador.")
            except ValueError:
                print("   [ERROR] Las coordenadas deben ser números. Inténtelo de nuevo.")
                
        elif opcion == '5':
            try:
                grados = float(input("   Ángulo hacia abajo (0=Frente, 90=Suelo) [45]: ") or "45")
                nodo.publicar_json(nodo.pub_camara, {"angulo": grados})
                print(f"   [OK] Orden de giro a {grados}° enviada.")
            except ValueError:
                print("   [ERROR] Introduce un número válido.")
                
        elif opcion == '6':
            print("\n   >>  Enviando orden de POBLAR...")
            nodo.publicar_accion("POBLAR")
            
        elif opcion == '7':
            print("\n   >>  Enviando orden de VACIAR...")
            nodo.publicar_accion("VACIAR")
            
        elif opcion == '8':
            print("\n   >>  Enviando orden de GENERAR...")
            nodo.publicar_accion("GENERAR")

        elif opcion == '9':
            print("\n   >>  Enviando orden de TERMINAR...")
            nodo.publicar_accion("TERMINAR")

        elif opcion == '10':
            print("\n   >>  Apagando el Orquestador y saliendo...")
            nodo.publicar_accion("SALIR")
            break # Rompe el bucle while y termina este script
        
        else:
            print("   [!] Opción no válida. Escribe un número del 1 al 10.")


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
