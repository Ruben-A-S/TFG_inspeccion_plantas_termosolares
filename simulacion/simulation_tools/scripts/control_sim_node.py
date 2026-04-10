import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import subprocess
import os

class SimulationControlNode(Node):
    def __init__(self):
        super().__init__('simulation_control_noed')

        # --- ESTADO INTERNO ---
        self.config_mundo = {"nombre": "mundo_prueba", "textura": "none"}  # datos por defecto
        self.config_paneles = {}
        self.config_dron = {}
        self.proceso_simulacion = None # Aquí guardaremos el proceso de PX4/Gazebo

        # --- PUBLICADORES (Para hablar con la Interfaz) ---
        self.pub_estado = self.create_publisher(String, '/sim_status/estado', 10)
        self.pub_log = self.create_publisher(String, '/sim_status/log', 10)

        # --- SUBSCRIPTORES (Para escuchar a la Interfaz) ---
        self.create_subscription(String, '/sim_cmd/config_mundo', self.cb_config_mundo, 10)
        self.create_subscription(String, '/sim_cmd/config_paneles', self.cb_config_paneles, 10)
        self.create_subscription(String, '/sim_cmd/config_dron', self.cb_config_dron, 10)
        self.create_subscription(String, '/sim_cmd/accion', self.cb_accion, 10)

        self.enviar_log("Nodo Orquestador Iniciado. Esperando configuraciones...")
        self.cambiar_estado("ESPERANDO_DATOS")

    # ==========================================
    # CALLBACKS DE CONFIGURACIÓN
    # ==========================================
    def cb_config_mundo(self, msg):
        self.config_mundo = json.loads(msg.data)
        self.enviar_log(f"Configuración de mundo recibida: {self.config_mundo['nombre']} (textura: {self.config_mundo['textura']})")

    def cb_config_paneles(self, msg):
        self.config_paneles = json.loads(msg.data)
        self.enviar_log(f"Ruta de paneles recibida: {self.config_paneles['ruta_csv']}")

    def cb_config_dron(self, msg):
        self.config_dron = json.loads(msg.data)
        self.enviar_log(f"Configuración de dron recibida: {self.config_dron['modelo']}")

    # ==========================================
    # CALLBACK DE ACCIONES PRINCIPALES
    # ==========================================
    def cb_accion(self, msg):
        orden = msg.data.upper()
        
        if orden == "GENERAR":
            self.ejecutar_generacion_mundo()
        elif orden == "LANZAR":
            self.ejecutar_lanzamiento()
        elif orden == "CERRAR" or orden == "ABORTAR":
            self.cerrar_simulacion()
        else:
            self.enviar_log(f"Orden desconocida: {orden}")

    # ==========================================
    # LÓGICA DE NEGOCIO (Los "Músculos")
    # ==========================================
    def ejecutar_generacion_mundo(self):
        if not self.config_mundo:
            self.enviar_log("ERROR: No se puede generar sin recibir datos del mundo.")
            return

        self.cambiar_estado("GENERANDO_MUNDO")
        self.enviar_log("Iniciando creación del archivo .sdf...")
        
        # Aquí usted llamaría a las funciones de generador_mundo.py que ya tiene hechas
        # Pasándole: self.config_mundo['nombre'], self.config_mundo['textura']
        
        self.enviar_log("Mundo SDF generado con éxito.")
        self.cambiar_estado("MUNDO_LISTO")

    def ejecutar_lanzamiento(self):
        self.cambiar_estado("ARRANCANDO_SIMULACION")
        self.enviar_log("Lanzando PX4 y Gazebo en segundo plano...")

        # Aquí preparamos el comando mágico que aprendimos
        ruta_mundo = f"~/Carpeta_TFG_Provisional/src/TFG_inspeccion_plantas_termosolares/simulacion/simulation_tools/worlds/{self.config_mundo.get('nombre', 'default')}.sdf"
        modelo_dron = self.config_dron.get('modelo', 'x500')

        comando = f"export PX4_GZ_WORLD={ruta_mundo} && export PX4_GZ_MODEL={modelo_dron} && cd ~/PX4-Autopilot && make px4_sitl gz_{modelo_dron}"
        
        # Lanzamos el proceso sin bloquear a ROS 2
        self.proceso_simulacion = subprocess.Popen(comando, shell=True, executable='/bin/bash')
        
        self.enviar_log("Simulador lanzado. Procediendo a inyectar paneles...")
        # Aquí llamaría a sus scripts de inyectar paneles CSV...
        
        self.cambiar_estado("SIMULACION_CORRIENDO")

    def cerrar_simulacion(self):
        self.enviar_log("Cerrando simulador y limpiando procesos...")
        if self.proceso_simulacion:
            # Comando duro para matar Gazebo y PX4 en Linux
            subprocess.run("killall -9 ruby px4 gz", shell=True)
            self.proceso_simulacion = None
        
        self.cambiar_estado("ESPERANDO_DATOS")
        self.enviar_log("Simulación abortada/cerrada.")

    # ==========================================
    # UTILIDADES
    # ==========================================
    def enviar_log(self, texto):
        msg = String()
        msg.data = texto
        self.pub_log.publish(msg)
        self.get_logger().info(texto) # También lo imprime en la terminal del nodo

    def cambiar_estado(self, nuevo_estado):
        msg = String()
        msg.data = nuevo_estado
        self.pub_estado.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    nodo = SimulationControlNode()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
