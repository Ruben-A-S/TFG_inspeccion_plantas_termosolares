import json
import os
import subprocess
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

# Importamos script externo generador de mundos
from world_generator import crear_mundo_base


class SimOrchestratorNode(Node):
    """
    Nodo Orquestador de la simulación.
    
    Se encarga de almacenar la configuración enviada por el usuario,
    generar el mundo, lanzar PX4 SITL con Gazebo, y enviar las órdenes
    de carga/descarga de paneles solares a los nodos correspondientes.
    """

    def __init__(self):
        super().__init__('sim_orchestrator_node')

        # --- ESTADO INTERNO ---
        self.config_fecha = {"fecha": "10/02/2001", "hora": "12:34"}
        self.config_mundo = {"nombre": "prueba1", "textura": "arenosillo.png"}
        self.config_paneles = {"modelo": "panel", "ruta_csv": "Crescent_Dunes.csv"}
        self.config_dron = {"modelo": "x500", "x": 0.0, "y": 0.0}
        self.proceso_simulacion = None 

        self.mundo_generado = {"nombre": "prueba1"}
        self.paneles_generados = {"ruta_csv": "mapa_3.txt"} 
        
        # --- PUBLICADORES ---
        self.pub_gestion_mapa = self.create_publisher(String, '/sim_cmd/map_management', 10)
        
        self.pub_estado = self.create_publisher(String, '/sim_status/state', 10)
        
        self.pub_log = self.create_publisher(String, '/sim_status/log', 10)
        
        self.pub_sim_activa = self.create_publisher(String, '/sim_status/active_sim', 10)
        
        self.pub_params_control = self.create_publisher(Float64MultiArray, '/control_param', 10)

        # --- SUSCRIPTORES ---
        self.create_subscription(String, '/sim_cmd/date_config', self.config_fecha_callback, 10)
        
        self.create_subscription(String, '/sim_cmd/world_config', self.config_mundo_callback, 10)
        
        self.create_subscription(String, '/sim_cmd/panel_config', self.config_paneles_callback, 10)
        
        self.create_subscription(String, '/sim_cmd/drone_config', self.config_dron_callback, 10)
        
        self.create_subscription(String, '/sim_cmd/action', self.accion_callback, 10)
        
        self.create_subscription(String, '/sim_cmd/rotate_camera', self.rotate_camera_callback, 10)
        
        self.create_subscription(String, '/sim_cmd/rotate_panel', self.rotate_panel_callback, 10)

        self.enviar_log("Nodo Orquestador Iniciado. Esperando configuraciones...")
        self.cambiar_estado("ESPERANDO_DATOS")

    # ==========================================
    # CALLBACKS DE RECUPERACIÓN DE DATOS
    # ==========================================
    
    def config_fecha_callback(self, msg):
        """Actualiza la fecha y hora interna."""
        try:
            self.config_fecha = json.loads(msg.data)
            fecha = self.config_fecha.get('fecha')
            hora = self.config_fecha.get('hora')
            self.enviar_log(f"Configuración de fecha actualizada: {fecha} a las {hora}")
        except json.JSONDecodeError:
            self.enviar_log("ERROR: JSON de fecha inválida.")
            
    def config_mundo_callback(self, msg):
        """Actualiza el nombre y textura del mundo a generar."""
        try:
            self.config_mundo = json.loads(msg.data)
            nombre = self.config_mundo.get('nombre')
            textura = self.config_mundo.get('textura')
            self.enviar_log(f"Configuración de mundo actualizada: {nombre} (textura: {textura})")
        except json.JSONDecodeError:
            self.enviar_log("ERROR: JSON de mundo inválido.")

    def config_paneles_callback(self, msg):
        """Actualiza la configuración de generación de paneles."""
        try:
            self.config_paneles = json.loads(msg.data)
            ruta = self.config_paneles.get('ruta_csv')
            modelo = self.config_paneles.get('modelo')
            self.enviar_log(f"Configuración de paneles actualizada: {ruta} (modelo: {modelo})")
        except json.JSONDecodeError:
            self.enviar_log("ERROR: JSON de paneles inválido.")

    def config_dron_callback(self, msg):
        """Actualiza el modelo de dron y su posición de despegue."""
        try:
            self.config_dron = json.loads(msg.data)
            modelo = self.config_dron.get('modelo')
            pos_x = self.config_dron.get('x')
            pos_y = self.config_dron.get('y')
            self.enviar_log(f"Configuración de dron actualizada: {modelo} en X={pos_x}, Y={pos_y}")
        except json.JSONDecodeError:
            self.enviar_log("ERROR: JSON de dron inválido.")
    
    def rotate_camera_callback(self, msg):
        """Actualiza el pitch de la cámara del dron."""
        try:
            datos = json.loads(msg.data)
            grados = datos.get("angulo", 45.0)
            
            # Convertimos a radianes aquí para que la calculadora reciba el dato listo
            radianes = grados * (3.14159265 / 180.0)
            
            # Preparamos el mensaje para la calculadora (Float64MultiArray)
            msg_control = Float64MultiArray()
            # [Angulo, Focal (por defecto 1.5), Distorsión (0.0)]
            msg_control.data = [float(radianes), 1.5, 0.0]        
            self.pub_params_control.publish(msg_control)
            
            self.enviar_log(f"Cámara movida a {grados} grados ({radianes:.3f} rad)")
            
        except Exception as e:
            self.enviar_log(f"ERROR al procesar ángulo de cámara: {e}")
    
    def rotate_panel_callback(self, msg):
        """Escucha el comando de girar panel para registrarlo en el log global."""
        try:
            datos = json.loads(msg.data)
            id_panel = datos.get("id_panel", "panel_0") 
            
            # Leemos la faceta para que el log sea exacto
            id_faceta = datos.get("id_faceta", "todas") 
            
            if "_f" in id_faceta:
                roll_inc_grados = datos.get("roll_inc", 0.0)
                pitch_inc_grados = datos.get("pitch_inc", 0.0)
            
                # Actualizamos el texto para reflejar si giramos todo o solo una pieza
                self.enviar_log(
                    f"Orden recibida: Girar {id_panel} (Faceta: {id_faceta}) "
                    f"(Roll: +{roll_inc_grados}º, Pitch: +{pitch_inc_grados}º). "
                    f"Delegando ejecución al Gestor de Mapa."
                )
                
            else:
                yaw_inc_grados = datos.get("yaw_inc", 0.0)
                pitch_inc_grados = datos.get("pitch_inc", 0.0)
            
                # Actualizamos el texto para reflejar si giramos todo o solo una pieza
                self.enviar_log(
                    f"Orden recibida: Girar {id_panel} (Faceta: {id_faceta}) "
                    f"(Yaw: +{yaw_inc_grados}º, Pitch: +{pitch_inc_grados}º). "
                    f"Delegando ejecución al Gestor de Mapa."
                )
            
        except Exception as e:
            self.enviar_log(f"ERROR al procesar giro de panel en el Orquestador: {e}")

    # ==========================================
    # CALLBACK DE ACCIONES PRINCIPALES
    # ==========================================
    
    def accion_callback(self, msg):
        """Recibe una orden de acción principal (GENERAR, POBLAR, etc.)."""
        orden = msg.data.upper()
        
        if orden == "GENERAR":
            self.ejecutar_generacion_total()
        elif orden == "POBLAR":
            self.inyectar_obstaculos()
        elif orden == "VACIAR":
            self.eliminar_obstaculos()
        elif orden == "TERMINAR":
            self.cerrar_simulacion()
        elif orden == "SALIR":
            self.enviar_log("Recibida orden de salida total. Limpiando...")
            self.cerrar_simulacion()
            raise SystemExit  
        else:
            self.enviar_log(f"Orden desconocida: {orden}")

    # ==========================================
    # GENERACION DE MUNDO
    # ==========================================
    
    def ejecutar_generacion_total(self):
        """Inicia Gazebo y PX4 SITL con las configuraciones actuales."""
        if self.proceso_simulacion is not None:
            self.enviar_log(
                "ADVERTENCIA: La simulación ya está corriendo. "
                "Cierra la actual (Opción 10) antes de generar otra."
            )
            return
            
        self.cambiar_estado("ARRANCANDO_SIMULACION")
        self.enviar_log("Fase 1: Preparando mundo virtual...")
        
        nombre_mundo = self.config_mundo.get('nombre', 'prueba1')
        nombre_textura = self.config_mundo.get('textura', 'arenosillo.png')
        
        # Rutas hardcodeadas (se podrían extraer a parámetros de ROS 2 en el futuro)
        base_dir = os.path.expanduser("~/Carpeta_TFG_Provisional/src/TFG_inspeccion_plantas_termosolares")
        ruta_mundo_original = os.path.join(base_dir, "simulacion/simulacion/worlds", f"{nombre_mundo}.sdf")
        ruta_textura = os.path.join(base_dir, "simulacion/simulacion/models/textures", nombre_textura)
        
        try:
            crear_mundo_base(nombre_mundo, ruta_textura, ruta_mundo_original)
            self.enviar_log(f"Mundo '{nombre_mundo}' generado exitosamente.")
        except Exception as e:
            self.enviar_log(f"ERROR al generar el mundo: {e}")
            return 
            
        self.enviar_log("Fase 2: Preparando rutas para PX4...")
        
        modelo_dron = self.config_dron.get('modelo', 'x500')
        pos_x = self.config_dron.get('x', 0.0)
        pos_y = self.config_dron.get('y', 0.0)

        ruta_px4_worlds = os.path.expanduser("~/PX4-Autopilot/Tools/simulation/gz/worlds")
        ruta_mundo_destino = os.path.join(ruta_px4_worlds, f"{nombre_mundo}.sdf")

        if os.path.exists(ruta_mundo_original):
            self.enviar_log("Copiando mundo a entorno PX4...")
            subprocess.run(f"cp {ruta_mundo_original} {ruta_mundo_destino}", shell=True)
        else:
            self.enviar_log(f"ADVERTENCIA: No se encontró el archivo {ruta_mundo_original}.")

        comando = (
            f"export PX4_GZ_WORLD={nombre_mundo} && "
            f"export PX4_GZ_MODEL_POSE='{pos_x},{pos_y},0.5,0,0,0' && "
            f"cd ~/PX4-Autopilot && make px4_sitl gz_{modelo_dron}"
        )
        
        self.enviar_log("Fase 3: Lanzando simulación...")
        self.proceso_simulacion = subprocess.Popen(
            comando, shell=True, executable='/bin/bash'
        )
        
        self.cambiar_estado("SIMULACION_CORRIENDO")
        self.mundo_generado = {"nombre": nombre_mundo}
        
        config_activa = {
            "mundo": nombre_mundo,
            "dron": modelo_dron
        }
        msg_activa = String()
        msg_activa.data = json.dumps(config_activa)
        self.pub_sim_activa.publish(msg_activa)
        
    def cerrar_simulacion(self):
        """Mata los procesos de Gazebo y PX4."""
        self.enviar_log("Cerrando simulador y limpiando procesos de Linux...")
        # Usa stderr=subprocess.DEVNULL para ocultar errores si no hay procesos que matar
        subprocess.run("killall -9 ruby px4 gz", shell=True, stderr=subprocess.DEVNULL)
        self.proceso_simulacion = None
        self.cambiar_estado("ESPERANDO_DATOS")
        self.enviar_log("Simulador cerrado.")

    # ==========================================
    # GESTIÓN DE PANELES
    # ==========================================
    
    def inyectar_obstaculos(self):
        """Pide al nodo de carga de mapas que inserte los paneles solares."""
        fecha_mundo = self.config_fecha.get('fecha', '10/02/2001')
        hora_mundo = self.config_fecha.get('hora', '12:34')
        nombre_csv = self.config_paneles.get('ruta_csv', 'mapa_3.txt')
        modelo_panel = self.config_paneles.get('modelo', 'panel')
        nombre_mundo = self.mundo_generado.get('nombre', 'prueba1')
        
        # 1. Actualizamos el estado interno
        self.paneles_generados = {"ruta_csv": nombre_csv}
        
        # 2. Empaquetamos la orden
        orden = {
            "accion": "CARGAR",
            "fecha": fecha_mundo,
            "hora": hora_mundo,
            "csv": nombre_csv,
            "modelo": modelo_panel,
            "mundo": nombre_mundo
        }
        
        # 3. Enviamos la orden al nodo load_map
        msg = String()
        msg.data = json.dumps(orden)
        self.pub_gestion_mapa.publish(msg)
        
        self.enviar_log(f"Orden enviada a load_map para poblar '{nombre_mundo}' con '{nombre_csv}'.")
        
    def eliminar_obstaculos(self):
        """Pide al nodo de carga de mapas que elimine los paneles solares."""
        nombre_csv = self.paneles_generados.get('ruta_csv', 'mapa_3.txt')
        nombre_mundo = self.mundo_generado.get('nombre', 'prueba1')

        orden = {
            "accion": "VACIAR",
            "csv": nombre_csv,
            "mundo": nombre_mundo
        }

        msg = String()
        msg.data = json.dumps(orden)
        self.pub_gestion_mapa.publish(msg)
        
        self.enviar_log(f"Orden enviada a load_map para vaciar el mapa '{nombre_csv}'.")
        self.paneles_generados = {}

    # ==========================================
    # UTILIDADES
    # ==========================================
    
    def enviar_log(self, texto):
        """Publica un mensaje en el tópico de logs y lo imprime localmente."""
        msg = String()
        msg.data = texto
        self.pub_log.publish(msg)
        self.get_logger().info(texto)

    def cambiar_estado(self, nuevo_estado):
        """Actualiza el estado global de la simulación."""
        msg = String()
        msg.data = nuevo_estado
        self.pub_estado.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    nodo = SimOrchestratorNode()
    
    try:
        rclpy.spin(nodo)
    except SystemExit:
        nodo.get_logger().info("Apagado del nodo solicitado por el usuario. Adiós.")
    except KeyboardInterrupt:
        nodo.get_logger().info("Apagado mediante Ctrl+C.")
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
