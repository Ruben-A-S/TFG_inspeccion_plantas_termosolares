import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import subprocess
import os
import sys

# Importamos su script externo generador de mundos
from world_generator import crear_mundo_base

class SimulationControlNode(Node):
    def __init__(self):
        super().__init__('simulation_control_node')

        # --- ESTADO INTERNO ---
        self.config_mundo = {"nombre": "prueba1", "textura": "arenosillo.png"}
        self.config_paneles = {"modelo": "panel", "ruta_csv": "mapa_3.txt"}
        self.config_dron = {"modelo": "x500", "x": 0.0, "y": 0.0}
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
    # CALLBACKS DE RECUPERACIÓN DE DATOS (BLINDADOS)
    # ==========================================
    def cb_config_mundo(self, msg):
        try:
            self.config_mundo = json.loads(msg.data)
            self.enviar_log(f"Configuración de mundo actualizada: {self.config_mundo.get('nombre')} (textura: {self.config_mundo.get('textura')})")
        except json.JSONDecodeError:
            self.enviar_log("ERROR: JSON de mundo inválido. Use comillas dobles: {\"nombre\": \"...\"}")

    def cb_config_paneles(self, msg):
        try:
            self.config_paneles = json.loads(msg.data)
            self.enviar_log(f"Configuración de paneles actualizada: {self.config_paneles.get('ruta_csv')} (modelo: {self.config_paneles.get('modelo')})")
        except json.JSONDecodeError:
            self.enviar_log("ERROR: JSON de paneles inválido. Use comillas dobles.")

    def cb_config_dron(self, msg):
        try:
            self.config_dron = json.loads(msg.data)
            self.enviar_log(f"Configuración de dron actualizada: {self.config_dron.get('modelo')} en posición X={self.config_dron.get('x')}, Y={self.config_dron.get('y')}")
        except json.JSONDecodeError:
            self.enviar_log("ERROR: JSON de dron inválido. Use comillas dobles.")
            
    # ==========================================
    # CALLBACK DE ACCIONES PRINCIPALES
    # ==========================================
    def cb_accion(self, msg):
        orden = msg.data.upper()
        
        if orden == "GENERAR":
            self.ejecutar_generacion_total()
        elif orden == "TERMINAR":
            self.cerrar_simulacion()
        elif orden == "SALIR":
            self.enviar_log("Recibida orden de salida total. Limpiando...")
            self.cerrar_simulacion()
            raise SystemExit  # Apaga el nodo ROS 2 limpiamente
        else:
            self.enviar_log(f"Orden desconocida: {orden}")

    # ==========================================
    # LÓGICA DE NEGOCIO (Los "Músculos")
    # ==========================================
    def ejecutar_generacion_total(self):
        self.cambiar_estado("ARRANCANDO_SIMULACION")
        
        self.enviar_log("Fase 1: Preparando mundo virtual...")
        
        # 1. Extraemos los datos de la configuración
        nombre_mundo = self.config_mundo.get('nombre', 'prueba1')
        nombre_textura = self.config_mundo.get('textura', 'arenosillo.png')
        
        # Rutas absolutas para el SDF y la textura
        ruta_mundo_original = os.path.expanduser(f"~/Carpeta_TFG_Provisional/src/TFG_inspeccion_plantas_termosolares/simulacion/simulation_tools/worlds/{nombre_mundo}.sdf")
        ruta_textura = os.path.expanduser(f"~/Carpeta_TFG_Provisional/src/TFG_inspeccion_plantas_termosolares/simulacion/simulation_tools/models/textures/{nombre_textura}")
        
        # Generar el archivo .sdf
        try:
            crear_mundo_base(nombre_mundo, ruta_textura, ruta_mundo_original)
            self.enviar_log(f"Mundo '{nombre_mundo}' generado exitosamente.")
        except Exception as e:
            self.enviar_log(f"ERROR al generar el mundo: {e}")
            return # Abortamos si falla la creación del mundo
            
        # 2. Preparar rutas para PX4
        self.enviar_log("Fase 2: Preparando rutas para PX4...")
        
        modelo_dron = self.config_dron.get('modelo', 'x500')
        pos_x = self.config_dron.get('x', '0.0')
        pos_y = self.config_dron.get('y', '0.0')

        # Ruta donde PX4 espera encontrar los mundos
        ruta_px4_worlds = os.path.expanduser("~/PX4-Autopilot/Tools/simulation/gz/worlds")
        ruta_mundo_destino = os.path.join(ruta_px4_worlds, f"{nombre_mundo}.sdf")

        # Copiamos el archivo al ecosistema de PX4
        if os.path.exists(ruta_mundo_original):
            self.enviar_log(f"Copiando mundo a entorno PX4...")
            subprocess.run(f"cp {ruta_mundo_original} {ruta_mundo_destino}", shell=True)
        else:
            self.enviar_log(f"ADVERTENCIA: No se encontró el archivo {ruta_mundo_original}. PX4 podría fallar.")

        # 3. Lanzar Gazebo y PX4
        comando = (
            f"export PX4_GZ_WORLD={nombre_mundo} && "
            f"export PX4_GZ_MODEL_POSE='{pos_x},{pos_y},0.5,0,0,0' && "
            f"cd ~/PX4-Autopilot && make px4_sitl gz_{modelo_dron}"
        )
        
        self.enviar_log(f"Fase 3: Lanzando simulación...")
        self.proceso_simulacion = subprocess.Popen(comando, shell=True, executable='/bin/bash')
        
        # 4. (Futuro) Inyectar los paneles con self.config_paneles
        self.enviar_log(f"Fase 4: (Pendiente) Inyectar paneles desde {self.config_paneles.get('ruta_csv')}...")
        
        self.cambiar_estado("SIMULACION_CORRIENDO")

    def cerrar_simulacion(self):
        self.enviar_log("Cerrando simulador y limpiando procesos de Linux...")
        
        # Comando duro para matar Gazebo y PX4
        subprocess.run("killall -9 ruby px4 gz", shell=True, stderr=subprocess.DEVNULL)
        self.proceso_simulacion = None
        
        self.cambiar_estado("ESPERANDO_DATOS")
        self.enviar_log("Simulador cerrado.")

    # ==========================================
    # UTILIDADES
    # ==========================================
    def enviar_log(self, texto):
        msg = String()
        msg.data = texto
        self.pub_log.publish(msg)
        self.get_logger().info(texto)

    def cambiar_estado(self, nuevo_estado):
        msg = String()
        msg.data = nuevo_estado
        self.pub_estado.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    nodo = SimulationControlNode()
    
    try:
        rclpy.spin(nodo)
    except SystemExit:
        nodo.get_logger().info("Apagado del nodo solicitado por el usuario. Adiós.")
    except KeyboardInterrupt:
        nodo.get_logger().info("Apagado mediante Ctrl+C.")
    finally:
        nodo.destroy_node()
        # Verificamos si rclpy sigue activo antes de apagarlo
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
