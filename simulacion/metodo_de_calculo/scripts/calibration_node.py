#!/usr/bin/env python3

import json
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import String, Float64MultiArray
from std_srvs.srv import Trigger

class CalibrationNode(Node):
    """
    Nodo encargado de aplicar el método matemático HelioPoint.
    Versión Industrial Segura + Soporte Multifacetas:
    - Alineación mediante Producto Cruzado Vectorial puro.
    - Media móvil para cancelación de defectos del espejo.
    - Soporta tanto paneles monolíticos como mallas de facetas.
    """

    def __init__(self):
        super().__init__('calibration_node')
        
        self.paneles_teoria = {} 
        self.historial_errores = {} 

        # --- CLIENTES Y SUSCRIPCIONES ---
        self.cli_teoria = self.create_client(Trigger, 'get_panel_theory')
        
        self.create_subscription(String, '/sim_status/panel_updates', self.actualizar_teoria_callback, 10)
        self.create_subscription(String, '/inspection/filtered_data', self.datos_filtrados_callback, 10)
        self.create_subscription(Float64MultiArray, '/control_param', self.param_callback, qos_profile_sensor_data)
        
        # --- PUBLICADORES ---
        self.pub_resultados = self.create_publisher(String, '/calibration/results', 10)
        self.error_pub = self.create_publisher(Float64MultiArray, '/heliostat_processed_errors', 10)

        self.panel_en_enfoque = None
        self.buffer_media_movil = [] 
        
        # Distancia de la cámara al LED (en el sistema local de la cámara)
        self.d_cam_led = np.array([0.0, 0.0, -0.6])  
        
        # Ángulo inicial por defecto (45 grados)
        self.angulo_cam = 0.785  

        self.pedir_mapa_teorico()
        self.get_logger().info("Cerebro HelioPoint iniciado [VERSIÓN FACETAS UNIFICADA]. Esperando teoría...")

    def pedir_mapa_teorico(self):
        if not self.cli_teoria.service_is_ready():
            return
        req = Trigger.Request()
        self.cli_teoria.call_async(req).add_done_callback(self.al_recibir_teoria)

    # ==========================================
    # LA MAGIA: CINEMÁTICA UNIFICADA EN MEMORIA
    # ==========================================
    def al_recibir_teoria(self, futuro):
        try:
            res = futuro.result()
            if res.success:
                lista = json.loads(res.message)
                self.paneles_teoria = {}
                
                for p in lista:
                    # Cinemática del panel base (Yaw -> Z, Pitch -> Y)
                    rot_yaw = R.from_euler('z', p.get('yaw', 0.0))
                    rot_pitch = R.from_euler('y', p.get('pitch', 0.0))
                    r_p_global = rot_yaw * rot_pitch
                    
                    pos_p_global = np.array([p.get('x', 0.0), p.get('y', 0.0), p.get('z', 0.0)])
                    
                    if 'facetas' in p:
                        for f in p['facetas']:
                            offset_local = np.array(f.get('offset', [0.0, 0.0, 0.0]))
                            pos_f = pos_p_global + r_p_global.apply(offset_local)
                            
                            # Cinemática local de la faceta (Roll -> X, Pitch -> Y)
                            rot_canting = R.from_euler('xy', [f.get('cant_roll', 0.0), f.get('cant_pitch', 0.0)])
                            r_f_final = r_p_global * rot_canting
                            
                            # Guardamos el objeto Rotation puro para no perder datos
                            self.paneles_teoria[str(f['id'])] = {
                                'pos': pos_f,
                                'rot': r_f_final
                            }
                    else:
                        self.paneles_teoria[str(p['id'])] = {
                            'pos': pos_p_global,
                            'rot': r_p_global
                        }
                        
                self.get_logger().info(f"Plano Teórico Aplanado: {len(self.paneles_teoria)} entidades rastreables.")
        except Exception as e:
            self.get_logger().error(f"Error cargando teoría: {e}")

    def actualizar_teoria_callback(self, msg):
        self.pedir_mapa_teorico()

    def param_callback(self, msg):
        if len(msg.data) >= 1: 
            self.angulo_cam = float(msg.data[0])

    def calcular_vector_medido(self, p_cam, r_dron_base, p_reflex):
        # r_dron_base es la rotación del dron. Añadimos el pitch de la cámara.
        r_cam_real = r_dron_base * R.from_euler('y', self.angulo_cam, degrees=False)
        p_led = p_cam + r_cam_real.apply(self.d_cam_led)
        
        v_reflejado = p_cam - p_reflex
        v_reflejado_unitario = v_reflejado / np.linalg.norm(v_reflejado)
        
        v_incidente = p_led - p_reflex
        v_incidente_unitario = v_incidente / np.linalg.norm(v_incidente)
        
        n_meas = v_incidente_unitario + v_reflejado_unitario
        return n_meas / np.linalg.norm(n_meas)

    def datos_filtrados_callback(self, msg):
        if not self.paneles_teoria:
            self.get_logger().warn("Esperando mapa teórico...", throttle_duration_sec=3.0)
            return 

        try:
            datos_filtrados = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error("Error: Mensaje JSON no válido.", throttle_duration_sec=3.0)
            return

        if not datos_filtrados: return

        resultados_iteracion = []

        for dato in datos_filtrados:
            id_panel = str(dato.get("id_panel"))
            
            # Buscamos la entidad en memoria
            panel_teo = self.paneles_teoria.get(id_panel)
            if not panel_teo:
                continue
                
            p_teo = panel_teo['pos']
            r_teo_original = panel_teo['rot']
            
            # El vector normal del espejo en reposo (como está tumbado en XY) es el eje Z
            n_teo_global = r_teo_original.apply([0.0, 0.0, 1.0])
            
            try:
                p_cam = np.array(dato["dron"]["pos"], dtype=float)
                quat_dron = np.array(dato["dron"]["quat"], dtype=float)
                p_rebote_local = np.array(dato["rebote_local"], dtype=float)
                
                if np.any(np.isnan(p_cam)) or np.any(np.isnan(quat_dron)) or np.any(np.isnan(p_rebote_local)):
                    continue
                    
                if np.linalg.norm(quat_dron) < 1e-6:
                    continue

                r_dron = R.from_quat(quat_dron)
                
            except (KeyError, ValueError, TypeError):
                continue
            
            r_iter = r_teo_original
            n_meas_global = np.array([0.0, 0.0, 0.0])
            
            # Newton-Raphson / Producto Cruzado para buscar la orientación real
            for _ in range(3):
                p_reflex_global = p_teo + r_iter.apply(p_rebote_local)
                n_meas_global = self.calcular_vector_medido(p_cam, r_dron, p_reflex_global)
                n_curr_global = r_iter.apply([0.0, 0.0, 1.0])
                
                cross_prod = np.cross(n_curr_global, n_meas_global)
                norm_cross = np.linalg.norm(cross_prod)
                
                if norm_cross > 1e-8:
                    axis = cross_prod / norm_cross
                    dot_prod = np.dot(n_curr_global, n_meas_global)
                    dot_prod = max(-1.0, min(1.0, dot_prod))
                    angle = math.acos(dot_prod)
                    r_corr = R.from_rotvec(axis * angle)
                    r_iter = r_corr * r_iter 
            
            n_final_global = r_iter.apply([0.0, 0.0, 1.0])
            
            # Deshacemos la rotación global para ver el error en el sistema local del espejo
            n_final_ccs = r_teo_original.inv().apply(n_final_global)
            
            # Extraemos el error. 
            # Error Rotación X (Roll) y Error Rotación Y (Pitch)
            error_rotX_rad = -math.atan2(n_final_ccs[1], n_final_ccs[2])
            error_rotY_rad = math.atan2(n_final_ccs[0], n_final_ccs[2])
            
            rotX_mrad = error_rotX_rad * 1000.0
            rotY_mrad = error_rotY_rad * 1000.0
            
            MAX_MUESTRAS = 50  
            
            if id_panel not in self.historial_errores:
                self.historial_errores[id_panel] = {"rotX": [], "rotY": []}
                
            self.historial_errores[id_panel]["rotX"].append(rotX_mrad)
            self.historial_errores[id_panel]["rotY"].append(rotY_mrad)
            
            if len(self.historial_errores[id_panel]["rotX"]) > MAX_MUESTRAS:
                self.historial_errores[id_panel]["rotX"].pop(0)
                self.historial_errores[id_panel]["rotY"].pop(0)
                
            media_rotX = float(np.mean(self.historial_errores[id_panel]["rotX"]))
            media_rotY = float(np.mean(self.historial_errores[id_panel]["rotY"]))
            muestras = len(self.historial_errores[id_panel]["rotX"])
            
            resultados_iteracion.append({
                "id": id_panel, 
                "muestras_tomadas": muestras,
                "error_actual_rotX_mrad": float(rotX_mrad),
                "error_actual_rotY_mrad": float(rotY_mrad),
                "error_x_mrad": media_rotX, 
                "error_y_mrad": media_rotY,
                "normal_teorica": n_teo_global.tolist(),
                "normal_medida": n_meas_global.tolist(),
                "rebote_local": p_rebote_local.tolist()
            })

        if resultados_iteracion:
            msg_pub = String(data=json.dumps(resultados_iteracion))
            self.pub_resultados.publish(msg_pub)

def main(args=None):
    rclpy.init(args=args)
    nodo = CalibrationNode()
    try: 
        rclpy.spin(nodo)
    except KeyboardInterrupt: 
        pass
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
