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

# Scripts externos de inyección
try:
    from Add_panels_from_file import inyectar_paneles
    from Remove_panels_from_file import eliminar_paneles
except ImportError:
    def inyectar_paneles(m, p, mod): pass
    def eliminar_paneles(m, p): pass

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

        self.enviar_log("Map Loader LISTO. Esperando 'id_panel' en /rotate_panel.")

    def get_panel_theory_callback(self, request, response):
        response.success = True
        response.message = json.dumps(self.paneles_teoria)
        return response

    def get_panel_real_callback(self, request, response):
        response.success = True
        response.message = json.dumps(self.paneles_realidad)
        return response

    def rotate_panel_callback(self, msg):
        """
        Maneja el mensaje: {"id_panel": "panel_0", "yaw_inc": 20.0, "pitch_inc": 10.0}
        """
        try:
            datos = json.loads(msg.data)
            target_id = datos.get("id_panel")  # CAMBIO: id_panel en lugar de id
            
            # Buscamos el panel (ahora soportando que el CSV empiece en 0 o 1)
            panel_real = next((p for p in self.paneles_realidad if p['id'] == target_id), None)
            
            if not panel_real:
                self.enviar_log(f"ERROR: El panel '{target_id}' no existe en memoria.")
                return

            # 1. Modificar la Realidad (en radianes)
            panel_real['yaw'] += math.radians(datos.get("yaw_inc", 0.0))
            panel_real['pitch'] += math.radians(datos.get("pitch_inc", 0.0))
            
            # 2. Actualizar Gazebo
            eliminar_paneles(self.mundo_actual, [panel_real])
            inyectar_paneles(self.mundo_actual, [panel_real], self.modelo_actual)
            
            # 3. Notificar al Faker
            update_msg = String()
            update_msg.data = json.dumps([target_id])
            self.pub_updates.publish(update_msg)
            
            self.enviar_log(f"ROTACIÓN: {target_id} movido exitosamente.")
            
        except Exception as e:
            self.enviar_log(f"Fallo en rotación: {e}")

    def gestion_mapa_callback(self, msg):
        try:
            datos = json.loads(msg.data)
            if datos.get("accion") == "CARGAR":
                self.mundo_actual = datos.get("mundo", self.mundo_actual)
                self.modelo_actual = datos.get("modelo", self.modelo_actual)
                
                # Cargamos
                self.paneles_teoria = self.generar_array_desde_csv(
                    datos.get("csv"), 
                    datos.get("fecha", "10/02/2001"), 
                    datos.get("hora", "12:00")
                )
                self.paneles_realidad = json.loads(json.dumps(self.paneles_teoria))
                
                # Inyectar
                inyectar_paneles(self.mundo_actual, self.paneles_realidad, self.modelo_actual)
                
                # Notificar al Faker
                update_msg = String()
                update_msg.data = json.dumps([p['id'] for p in self.paneles_realidad])
                self.pub_updates.publish(update_msg)
                self.enviar_log(f"MAPA CARGADO: {len(self.paneles_teoria)} paneles.")

            elif datos.get("accion") == "VACIAR":
                eliminar_paneles(self.mundo_actual, self.paneles_realidad)
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
                next(f) # Saltar primera linea de metadatos
                lector = csv.DictReader(f)
                for fila in lector:
                    if len(lista) >= 5: break 
                    
                    x, y, z = float(fila["Heliostat x"]), float(fila["Heliostat y"]), float(fila["Heliostat z"])
                    ax, ay, az = float(fila["Aiming point x"]), float(fila["Aiming point y"]), float(fila["Aiming point z"])
                    
                    yaw, pitch = calcular_orientacion_heliostato([x,y,z], [ax,ay,az], obtener_sol_inventado(fecha, hora))
                    
                    # CAMBIO: Empezamos en panel_0 para que coincida con tu CLI
                    panel_id = f"panel_{len(lista)}"
                    
                    lista.append({
                        "id": panel_id, 
                        "x": x, "y": y, "z": z + 5,
                        "yaw": yaw, "pitch": pitch,
                        "width_x": float(fila["Heliostat width (x)"]),
                        "length_y": float(fila["Heliostat length (y)"])
                    })
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
        rclpy.shutdown()

if __name__ == '__main__': main()
