#!/usr/bin/env python3

import csv
import json
import os
import math
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration

from scipy.spatial.transform import Rotation as R

# Scripts externos de inyección
try:
    from Add_panels_from_file import inyectar_paneles
    from Remove_panels_from_file import eliminar_paneles
except ImportError:
    def inyectar_paneles(m, p, mod): pass
    def eliminar_paneles(m, p): pass

# ==========================================
# CONFIGURACIÓN GLOBAL
# ==========================================
# Cambia esto a False para usar los paneles simples originales
MODO_AVANZADO = True 

# ==========================================
# FUNCIONES MATEMÁTICAS
# ==========================================

def obtener_sol_inventado(fecha, hora):
    return [1000.0, 100.0, 500.0]

def calcular_orientacion_heliostato(p_c, p_aim, p_s):
    """
    La normal teórica de cada espejo será la bisectriz
    entre la direccion que une éste con el sol y con 
    el punto al que debe apuntar.
    """
    v_dl = -np.array(p_s) + np.array(p_c)
    d_dl = v_dl / np.linalg.norm(v_dl)
    v_rl = np.array(p_aim) - np.array(p_c)
    d_rl = v_rl / np.linalg.norm(v_rl)
    n = d_rl - d_dl
    n = n / np.linalg.norm(n)
    yaw = np.arctan2(n[1], n[0])
    pitch = np.arcsin(n[2])
    return float(yaw), float(pitch)

# ==========================================
# NODO MAP_LOADER_NODE
# ==========================================

class MapLoaderNode(Node):
    """
    Nodo para la gestión de los paneles

    Incluye el cálculo de la dirección teórica para éstos y 
    la gestión de las órdenes de giro dadas por la interfaz
    """
    def __init__(self):
        super().__init__('map_loader_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.paneles_teoria = []   
        self.paneles_realidad = [] 
        
        self.mundo_actual = "prueba1"
        self.modelo_actual = "panel"

        # --- SUSCRIPTORES ---
        self.create_subscription(String, '/sim_cmd/map_management', self.gestion_mapa_callback, 10)
        self.create_subscription(String, '/sim_cmd/rotate_panel', self.rotate_panel_callback, 10)

        # --- PUBLICADORES ---
        self.pub_log = self.create_publisher(String, '/sim_status/log', 10)
        self.pub_updates = self.create_publisher(String, '/sim_status/panel_updates', 10)

        # --- SERVICIOS ---
        self.srv_teoria = self.create_service(Trigger, 'get_panel_theory', self.get_panel_theory_callback)
        self.srv_realidad = self.create_service(Trigger, 'get_panel_real', self.get_panel_real_callback)
        
        # Timer para publicar TF a 20Hz (para ver en RViz)
        self.create_timer(0.05, self.broadcast_tf_tree)
        self.get_logger().info("Map Loader inicializado con soporte TF2.")

        modo_str = "AVANZADO (Facetas)" if MODO_AVANZADO else "SIMPLE (Bloque)"
        self.enviar_log(f"Map Loader LISTO en MODO {modo_str}. Esperando órdenes.")

    def broadcast_tf_tree(self):
        """
        Función para publicar los tf que permitan visualizar 
        los sistemas de referencia en RVIZ.
        """
        now = self.get_clock().now().to_msg()
        for panel in self.paneles_realidad:
            # 1. FRAME: Poste del panel (Yaw)
            # map -> panel_id
            t_panel = TransformStamped()
            t_panel.header.stamp = now
            t_panel.header.frame_id = 'world'
            t_panel.child_frame_id = f"panel_{panel['id']}"
            t_panel.transform.translation.x = float(panel['x'])
            t_panel.transform.translation.y = float(panel['y'])
            t_panel.transform.translation.z = float(panel['z'])
            q_p = R.from_euler('z', panel['yaw']).as_quat()
            t_panel.transform.rotation.x, t_panel.transform.rotation.y = q_p[0], q_p[1]
            t_panel.transform.rotation.z, t_panel.transform.rotation.w = q_p[2], q_p[3]
            self.tf_broadcaster.sendTransform(t_panel)

            # 2. FRAME: Plano inclinado del panel (Pitch)
            # panel_id -> panel_inclinado_id
            t_pitch = TransformStamped()
            t_pitch.header.stamp = now
            t_pitch.header.frame_id = f"panel_{panel['id']}"
            t_pitch.child_frame_id = f"inclinacion_{panel['id']}"
            # Rotamos sobre Y para aplicar el pitch del panel completo
            q_pitch = R.from_euler('y', panel['pitch']).as_quat()
            t_pitch.transform.rotation.x, t_pitch.transform.rotation.y = q_pitch[0], q_pitch[1]
            t_pitch.transform.rotation.z, t_pitch.transform.rotation.w = q_pitch[2], q_pitch[3]
            self.tf_broadcaster.sendTransform(t_pitch)

            # 3. FRAME: Cada faceta (Offset + ajuste fino)
            # inclinacion_id -> faceta_id
            for faceta in panel.get('facetas', []):
                t_faceta = TransformStamped()
                t_faceta.header.stamp = now
                t_faceta.header.frame_id = f"inclinacion_{panel['id']}"
                t_faceta.child_frame_id = f"faceta_{faceta['id']}"
                
                # Traslación (Offset)
                t_faceta.transform.translation.x = float(faceta['offset'][0])
                t_faceta.transform.translation.y = float(faceta['offset'][1])
                t_faceta.transform.translation.z = float(faceta['offset'][2])
                
                # Ajuste fino local (Canting) sobre X (Roll) e Y (Pitch)
                q_f = R.from_euler('xy', [faceta.get('cant_roll', 0.0), faceta.get('cant_pitch', 0.0)]).as_quat()
                t_faceta.transform.rotation.x, t_faceta.transform.rotation.y = q_f[0], q_f[1]
                t_faceta.transform.rotation.z, t_faceta.transform.rotation.w = q_f[2], q_f[3]
                self.tf_broadcaster.sendTransform(t_faceta)
                
    def get_panel_theory_callback(self, request, response):
        response.success = True
        response.message = json.dumps(self.paneles_teoria)
        return response

    def get_panel_real_callback(self, request, response):
        response.success = True
        response.message = json.dumps(self.paneles_realidad)
        return response

    def obtener_entidades_gazebo(self, lista_paneles):
        """
        Traduce nuestra memoria jerárquica a objetos planos para Gazebo.
        - Panel Global: Yaw (poste vertical Z) + Pitch (bisagra horizontal Y)
        - Faceta: Roll (Eje X) + Pitch (Eje Y) local
        """
        entidades_planas = []
        
        for panel in lista_paneles:
            if not MODO_AVANZADO:
                entidades_planas.append(panel)
                continue

            # 1. Base del panel en el mundo (Vector de traslación)
            pos_panel = np.array([panel['x'], panel['y'], panel['z']])
            
            # 2. CINEMÁTICA DEL PANEL GLOBAL
            # Construimos los "motores" de la estructura:
            rot_yaw = R.from_euler('z', panel['yaw'])     # Motor del poste vertical
            rot_pitch = R.from_euler('y', panel['pitch']) # Motor de la bisagra horizontal
            
            # Al multiplicar (Yaw * Pitch), Scipy aplica primero el Pitch sobre el Y local, 
            # y luego hace girar todo el conjunto sobre el Z vertical. 
            rot_global_marco = rot_yaw * rot_pitch

            for faceta in panel.get('facetas', []):
                # Desplazamiento de la faceta sobre la parrilla del panel
                offset_local = np.array(faceta['offset'])

                # A) CÁLCULO DE POSICIÓN ABSOLUTA
                # Colocamos el offset en el marco rotado y lo sumamos a la base
                pos_absoluta = pos_panel + rot_global_marco.apply(offset_local)

                # B) ROTACIÓN LOCAL DE LA FACETA (Canting)
                # Tal como indicaste, mantenemos esto igual porque los giros de facetas van bien
                cant_roll = faceta.get('cant_roll', 0.0)
                cant_pitch = faceta.get('cant_pitch', 0.0)
                rot_canting = R.from_euler('xy', [cant_roll, cant_pitch])
                
                # C) ROTACIÓN TOTAL Y EXTRACCIÓN
                # Sumamos la rotación de la estructura base + la rotación del propio espejo
                rot_absoluta = rot_global_marco * rot_canting

                # Extracción clásica idéntica a la que usaba el TF para pasárselo a Gazebo
                euler_final = rot_absoluta.as_euler('xyz')

                entidades_planas.append({
                    "id": faceta['id'],
                    "x": float(pos_absoluta[0]),
                    "y": float(pos_absoluta[1]),
                    "z": float(pos_absoluta[2]),
                    "roll": float(euler_final[0]),
                    "pitch": float(euler_final[1]), 
                    "yaw": float(euler_final[2])    
                })

        return entidades_planas
        
    def rotate_panel_callback(self, msg):
        try:
            datos = json.loads(msg.data)
            target_id = datos.get("id_panel")
            id_faceta = datos.get("id_faceta", "todas") 
            
            panel_real = next((p for p in self.paneles_realidad if p['id'] == target_id), None)
            
            if not panel_real:
                self.enviar_log(f"ERROR: El panel '{target_id}' no existe.")
                return

            # =========================================================
            # MODO FACETA ÚNICA
            # =========================================================
            if MODO_AVANZADO and id_faceta != "todas":
                # 1. Buscamos la faceta concreta en la memoria antes de cambiarla
                faceta_antigua = next((f for f in panel_real.get('facetas', []) if f['id'] == id_faceta), None)
                if not faceta_antigua:
                    self.enviar_log(f"ERROR: Faceta '{id_faceta}' no encontrada.")
                    return
                
                # 2. Calculamos su estado en Gazebo JUSTO ANTES del cambio para poder borrar SOLO esa faceta
                # Para ello, usamos una lista temporal con un "falso panel" que solo tiene esa faceta
                panel_temporal = panel_real.copy()
                panel_temporal['facetas'] = [faceta_antigua]
                entidad_vieja_unica = self.obtener_entidades_gazebo([panel_temporal])
                
                # Borramos SOLO esa faceta en Gazebo
                eliminar_paneles(self.mundo_actual, entidad_vieja_unica)

                # 3. Aplicamos el giro en memoria a la faceta
                faceta_antigua['cant_roll'] += math.radians(datos.get("roll_inc", 0.0))
                faceta_antigua['cant_pitch'] += math.radians(datos.get("pitch_inc", 0.0))

                # 4. Calculamos la nueva posición de SOLO esa faceta y la inyectamos
                entidad_nueva_unica = self.obtener_entidades_gazebo([panel_temporal])
                inyectar_paneles(self.mundo_actual, entidad_nueva_unica, "faceta")

                self.enviar_log(f"QUIRÚRGICO [OK]: Reemplazada únicamente la faceta {id_faceta}.")

            # =========================================================
            # MODO GLOBAL: MOVER EL PANEL ENTERO (TODAS LAS FACETAS)
            # =========================================================
            else:
                # Borramos todo el bloque/facetas del panel actual
                entidades_viejas = self.obtener_entidades_gazebo([panel_real])
                eliminar_paneles(self.mundo_actual, entidades_viejas)

                # Modificamos el tracking principal
                panel_real['yaw'] += math.radians(datos.get("yaw_inc", 0.0))
                panel_real['pitch'] += math.radians(datos.get("pitch_inc", 0.0))

                # Reinyectamos el panel completo actualizado
                entidades_nuevas = self.obtener_entidades_gazebo([panel_real])
                modelo_inyectar = "faceta" if MODO_AVANZADO else self.modelo_actual
                inyectar_paneles(self.mundo_actual, entidades_nuevas, modelo_inyectar)
                
                self.enviar_log(f"ROTACIÓN GLOBAL [OK]: {target_id} reorganizado por completo.")
            
            # 5. Notificar al sistema
            self.pub_updates.publish(String(data=json.dumps([target_id])))
            
        except Exception as e:
            self.enviar_log(f"Fallo en rotación: {e}")
            
    def gestion_mapa_callback(self, msg):
        try:
            datos = json.loads(msg.data)
            if datos.get("accion") == "CARGAR":
                self.mundo_actual = datos.get("mundo", self.mundo_actual)
                self.modelo_actual = datos.get("modelo", self.modelo_actual)
                
                self.paneles_teoria = self.generar_array_desde_csv(
                    datos.get("csv"), 
                    datos.get("fecha", "10/02/2001"), 
                    datos.get("hora", "12:00")
                )
                self.paneles_realidad = json.loads(json.dumps(self.paneles_teoria))
                
                # Traducimos a objetos planos y mandamos a Gazebo
                entidades = self.obtener_entidades_gazebo(self.paneles_realidad)
                modelo_inyectar = "faceta" if MODO_AVANZADO else self.modelo_actual
                inyectar_paneles(self.mundo_actual, entidades, modelo_inyectar)
                
                ids_notificacion = [p['id'] for p in self.paneles_realidad]
                self.pub_updates.publish(String(data=json.dumps(ids_notificacion)))
                self.enviar_log(f"MAPA CARGADO: {len(self.paneles_teoria)} paneles (Avanzado: {MODO_AVANZADO}).")

            elif datos.get("accion") == "VACIAR":
                entidades = self.obtener_entidades_gazebo(self.paneles_realidad)
                eliminar_paneles(self.mundo_actual, entidades)
                self.paneles_teoria = []
                self.paneles_realidad = []
                self.pub_updates.publish(String(data="[]"))
                self.enviar_log("MAPA VACIADO.")

        except Exception as e:
            self.enviar_log(f"Error gestion: {e}")

    def generar_array_desde_csv(self, nombre_csv, fecha, hora):
        """
        Inspeccionamos el csv para hallar los datos teóricos de los paneles
        """
        ruta = os.path.expanduser(f"~/{nombre_csv}")
        lista = []
        try:
            with open(ruta, mode='r', encoding='utf-8') as f:
                next(f) 
                lector = csv.DictReader(f)
                for fila in lector:
                    if len(lista) >= 5: break 
                    
                    x, y, z = float(fila["Heliostat x"]), float(fila["Heliostat y"]), float(fila["Heliostat z"])
                    ax, ay, az = float(fila["Aiming point x"]), float(fila["Aiming point y"]), float(fila["Aiming point z"])
                    
                    yaw, pitch = calcular_orientacion_heliostato([x,y,z], [ax,ay,az], obtener_sol_inventado(fecha, hora))
                    
                    panel_id = f"panel_{len(lista)}"
                    width_x = float(fila["Heliostat width (x)"])
                    length_y = float(fila["Heliostat length (y)"])
                    
                    # Diccionario base del panel
                    panel_data = {
                        "id": panel_id, 
                        "x": x, "y": y, "z": z + 5,
                        "yaw": yaw, "pitch": pitch,
                        "width_x": width_x,
                        "length_y": length_y
                    }

                    # --- MODO 2: GENERACIÓN DE 5x5 FACETAS ---
                    if MODO_AVANZADO:
                        facetas = []
                        w_faceta = width_x / 5.0
                        l_faceta = length_y / 5.0
                        
                        # Bucle de -2 a +2 para generar una cuadrícula centrada en el poste
                        for i in range(-2, 3):
                            for j in range(-2, 3):
                                id_faceta = f"{panel_id}_f{i+2}_{j+2}"
                                facetas.append({
                                    "id": id_faceta,
                                    "offset": [i * w_faceta, j * l_faceta, 0.0],
                                    "cant_roll": 0.0,
                                    "cant_pitch": 0.0
                                })
                        panel_data["facetas"] = facetas
                    
                    lista.append(panel_data)
                    
            return lista
        except Exception as e:
            self.enviar_log(f"Error CSV: {e}")
            return []

    def enviar_log(self, texto):
        msg = String(data=f"[MAP_LOADER] {texto}")
        self.pub_log.publish(msg)
        self.get_logger().info(texto)

def main(args=None):
    rclpy.init(args=args)
    nodo = MapLoaderNode()
    try: rclpy.spin(nodo)
    except KeyboardInterrupt: pass
    finally:
        nodo.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__': main()



