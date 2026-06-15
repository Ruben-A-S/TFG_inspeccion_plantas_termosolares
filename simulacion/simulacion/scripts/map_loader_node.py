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
    def __init__(self):
        super().__init__('map_loader_node')

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

        modo_str = "AVANZADO (Facetas)" if MODO_AVANZADO else "SIMPLE (Bloque)"
        self.enviar_log(f"Map Loader LISTO en MODO {modo_str}. Esperando órdenes.")

    def get_panel_theory_callback(self, request, response):
        response.success = True
        response.message = json.dumps(self.paneles_teoria)
        return response

    def get_panel_real_callback(self, request, response):
        response.success = True
        response.message = json.dumps(self.paneles_realidad)
        return response

    # ---------------------------------------------------------
    # LA MAGIA CINEMÁTICA: Aplanar la jerarquía para Gazebo
    # ---------------------------------------------------------
    def obtener_entidades_gazebo(self, lista_paneles):
        """
        Traduce nuestra memoria jerárquica (Panel -> Facetas) a una lista plana 
        de objetos que el script Add_panels_from_file puede meter en Gazebo.
        """
        entidades_planas = []
        
        for panel in lista_paneles:
            if not MODO_AVANZADO:
                # MODO 1: El panel se inyecta tal cual
                entidades_planas.append(panel)
            else:
                # MODO 2: Calculamos la posición 3D de sus 25 facetas en tiempo real
                p_centro_global = np.array([panel['x'], panel['y'], panel['z']])
                r_tracking = R.from_euler('zy', [panel['yaw'], panel['pitch']])
                
                for faceta in panel.get('facetas', []):
                    offset_local = np.array(faceta['offset'])
                    r_canting = R.from_euler('zy', [ faceta['cant_yaw'], faceta['cant_pitch']])
                    
                    # Efecto Tiovivo (Traslación Orbital)
                    p_faceta_global = p_centro_global + r_tracking.apply(offset_local)
                    
                    # Rotación Total (Motor central + Motor de la faceta)
                    r_final = r_tracking * r_canting
                    euler_final = r_final.as_euler('xyz')
                    
                    entidades_planas.append({
                        "id": faceta['id'],
                        "x": float(p_faceta_global[0]),
                        "y": float(p_faceta_global[1]),
                        "z": float(p_faceta_global[2]),
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
            # OPTIMIZACIÓN QUIRÚRGICA: MODO FACETA ÚNICA
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
                faceta_antigua['cant_yaw'] += math.radians(datos.get("yaw_inc", 0.0))
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
                                    "cant_yaw": 0.0,
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
