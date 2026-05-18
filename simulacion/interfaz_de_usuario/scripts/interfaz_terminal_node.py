import json
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class InterfazTerminalNode(Node):
    """
    Nodo publicador para enviar comandos desde la terminal.
    
    Publica en varios tópicos de '/sim_cmd/' para configurar y controlar
    la simulación del entorno.
    """

    def __init__(self):
        super().__init__('interfaz_terminal_node')
        
        # Publicadores
        self.pub_mundo = self.create_publisher(String, '/sim_cmd/world_config', 10)
        self.pub_fecha = self.create_publisher(String, '/sim_cmd/date_config', 10)
        self.pub_paneles = self.create_publisher(String, '/sim_cmd/panel_config', 10)
        self.pub_dron = self.create_publisher(String, '/sim_cmd/drone_config', 10)
        self.pub_accion = self.create_publisher(String, '/sim_cmd/action', 10)
        self.pub_camara = self.create_publisher(String, '/sim_cmd/rotate_camera', 10)
        self.pub_giro_panel =self.create_publisher(String, '/sim_cmd/rotate_panel', 10)
 
    def publicar_json(self, publicador, diccionario):
        """
        Convierte un diccionario a JSON y lo publica como String.
        """
        msg = String()
        msg.data = json.dumps(diccionario)
        publicador.publish(msg)

    def publicar_accion(self, accion):
        """
        Publica una acción en texto plano en el tópico correspondiente.
        """
        msg = String()
        msg.data = accion
        self.pub_accion.publish(msg)


def menu_interactivo(nodo):
    """
    Bucle infinito que lee comandos del teclado usando input()
    y publica los datos a través del nodo.
    """
    time.sleep(0.5)  # Pequeña pausa para asegurar que ROS 2 conecta

    while True:
        print("\n" + "=" * 45)
        print("   PANEL DE CONTROL DE SIMULACIÓN ")
        print("=" * 45)
        print("1.  Configurar Mundo (Nombre y Textura)")
        print("2.  Configurar Fecha y Hora")
        print("3.  Configurar Paneles (Archivo CSV y Modelo)")
        print("4.  Configurar Dron (Modelo y Posición)")
        print("-" * 45)
        print("5.  Girar Camara en vivo (Grados)")
        print("6.  Girar Paneles en vivo (Grados)")
        print("-" * 45)        
        print("7.  Poblar mundo")
        print("8.  Vaciar mundo")
        print("-" * 45)
        print("9.  LANZAR SIMULACIÓN (GENERAR)")
        print("10.  Detener Simulación (TERMINAR)")
        print("11. Apagar Todo y Salir (SALIR)")
        print("=" * 45)

        opcion = input(" Elige una opción (1-11): ")

        if opcion == '1':
            nombre = input("   Nombre del mundo [prueba1]: ") or "prueba1"
            textura = input("   Ruta de textura [none]: ") or "none"
            nodo.publicar_json(nodo.pub_mundo, {"nombre": nombre, "textura": textura})
            print("   [OK] Datos del mundo enviados al Orquestador.")

        elif opcion == '2':
            fecha = input("   Fecha (ej. 10/02/2001) [10/02/2001]: ") or "10/02/2001"
            hora = input("   Hora (ej. 12:34) [12:34]: ") or "12:34"
            nodo.publicar_json(nodo.pub_fecha, {"fecha": fecha, "hora": hora})
            print("   [OK] Datos de fecha y hora enviados al Orquestador.")

        elif opcion == '3':
            ruta = input("   Ruta del CSV [Crescent_Dunes.csv]: ") or "Crescent_Dunes.csv"
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
            try:
                id_panel = (input("   Id del panel a girar [panel_0]: ") or "panel_0")
                angulo_yaw = float(input("   incremento de yaw [0.0]: ") or "0.0")
                angulo_pitch = float(input("   incremento de pitch [0.0]: ") or "0.0")
                nodo.publicar_json(nodo.pub_giro_panel, {"id_panel": id_panel, "yaw_inc": angulo_yaw, "pitch_inc": angulo_pitch})
                print(f"   [OK] Orden a {id_panel} de giro yaw += {angulo_yaw}° y pitch += {angulo_pitch}º enviada.")
            except ValueError:
                print("   [ERROR] Introduce un id de panel y valores numéricos de ángulos válidos.")
                
        elif opcion == '7':
            print("\n   >>  Enviando orden de POBLAR...")
            nodo.publicar_accion("POBLAR")
            
        elif opcion == '8':
            print("\n   >>  Enviando orden de VACIAR...")
            nodo.publicar_accion("VACIAR")
            
        elif opcion == '9':
            print("\n   >>  Enviando orden de GENERAR...")
            nodo.publicar_accion("GENERAR")

        elif opcion == '10':
            print("\n   >>  Enviando orden de TERMINAR...")
            nodo.publicar_accion("TERMINAR")

        elif opcion == '11':
            print("\n   >>  Apagando el Orquestador y saliendo...")
            nodo.publicar_accion("SALIR")
            break
        
        else:
            print("   [!] Opción no válida. Escribe un número del 1 al 11.")


def main(args=None):
    rclpy.init(args=args)
    nodo = InterfazTerminalNode()

    # SEPARACION EN DOS HILOS:
    # ROS 2 gira en segundo plano para poder enviar mensajes
    hilo_ros = threading.Thread(target=rclpy.spin, args=(nodo,), daemon=True)
    hilo_ros.start()

    # El hilo principal se queda atrapado en los input() esperando al usuario
    try:
        menu_interactivo(nodo)
    except KeyboardInterrupt:
        print("\nSaliendo del panel por teclado (Ctrl+C).")
    finally:
        # Se asegura de limpiar el nodo y cerrar ROS 2 correctamente al salir
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
