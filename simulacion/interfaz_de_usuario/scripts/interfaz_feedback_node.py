#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Color:
    """Clase auxiliar para manejar colores en la salida de la terminal mediante ANSI."""
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'


class FeedbackMonitorNode(Node):
    """
    Nodo suscriptor para monitorizar el estado general de la simulación.
    
    Escucha los tópicos de log y estado del sistema y los imprime
    en la terminal con un formato de colores según su nivel de criticidad.
    """

    def __init__(self):
        # He cambiado el nombre interno del nodo para que sea más descriptivo
        super().__init__('feedback_monitor_node')
        
        # --- SUSCRIPCIONES ---
        # Escuchamos los logs generales del sistema
        self.create_subscription(
            String, 
            '/sim_status/log', 
            self.log_callback, 
            10
        )
        
        # Escuchamos el estado de la simulación
        self.create_subscription(
            String, 
            '/sim_status/estado', 
            self.estado_callback, 
            10
        )
        
        self.estado_actual = ""

        # Imprimimos el encabezado de inicio
        print(f"\n{Color.OKCYAN}{Color.BOLD}" + "=" * 50)
        print("       SISTEMA DE MONITOREO Y RESPUESTAS ACTIVO")
        print("=" * 50 + f"{Color.ENDC}\n")

    def estado_callback(self, msg):
        """Callback que se ejecuta cuando el orquestador cambia de estado."""
        nuevo_estado = msg.data
        if nuevo_estado != self.estado_actual:
            self.estado_actual = nuevo_estado
            print(f"{Color.BOLD}{Color.OKBLUE}[SISTEMA]{Color.ENDC} "
                  f"Estado cambiado a: {Color.BOLD}{nuevo_estado}{Color.ENDC}")

    def log_callback(self, msg):
        """Callback que procesa y clasifica por colores los logs de la simulación."""
        texto = msg.data
        
        # 1. ANALIZAR SI LAS ÓRDENES VAN BIEN O MAL
        if "ERROR" in texto or "FAIL" in texto or "incorrecto" in texto.lower():
            print(f"{Color.FAIL}{Color.BOLD}[¡ERROR!] {Color.ENDC}{Color.FAIL}{texto}{Color.ENDC}")
        
        elif "ÉXITO" in texto or "completad" in texto.lower() or "[OK]" in texto:
            print(f"{Color.OKGREEN}{Color.BOLD}[ÉXITO] {Color.ENDC}{texto}")

        # 2. AVISAR DE LOS DATOS DE DETECCIÓN (Cámara / ShowDataNode)
        elif "DESTELLO" in texto or "DETECTADO" in texto:
            # Resaltamos el destello con colores llamativos
            print(f"{Color.WARNING}{Color.BOLD}[CÁMARA] {texto}{Color.ENDC}")

        # 3. LOGS DE PROCESO NORMAL
        elif "Fase" in texto or "Iniciando" in texto:
            print(f"{Color.OKCYAN}[PROCESO]{Color.ENDC} {texto}")
        
        else:
            # Mensajes informativos generales
            print(f"  • {texto}")


def main(args=None):
    rclpy.init(args=args)
    nodo = FeedbackMonitorNode()
    
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        print("\nCerrando monitor de respuestas...")
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
