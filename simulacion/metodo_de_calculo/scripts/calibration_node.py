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
    Versión Industrial Segura:
    - Alineación mediante Producto Cruzado Vectorial puro (Evita Divergencia).
    - Protección contra datos corruptos (NaN, Inf, Zero Quaternions).
    - Media móvil para cancelación de defectos del espejo.
    - Lectura dinámica del ángulo del Gimbal de la cámara.
    """

    def __init__(self):
        super().__init__('calibration_node')
        
        self.paneles_teoria = {} 

        # --- CLIENTES Y SUSCRIPCIONES ---
        self.cli_teoria = self.create_client(Trigger, 'get_panel_theory')
        self.create_subscription(String, '/sim_status/panel_updates', self.actualizar_teoria_callback, 10)
        self.create_subscription(String, '/inspection/filtered_data', self.datos_filtrados_callback, 10)
        
        # NUEVO: Suscripción al ángulo de la cámara
        self.create_subscription(Float64MultiArray, '/control_param', self.param_callback, qos_profile_sensor_data)
        
        self.pub_resultados = self.create_publisher(String, '/calibration/results', 10)
        
        self.historial_errores = {}
        
        # Distancia de la cámara al LED (en el sistema local de la cámara)
        self.d_cam_led = np.array([0.0, 0.0, -0.6])  
        
        # Ángulo inicial por defecto (45 grados), se actualizará con el topic
        self.angulo_cam = 0.785  

        self.pedir_mapa_teorico()
        self.get_logger().info("Cerebro HelioPoint iniciado [VERSIÓN SEGURA]. Esperando teoría...")

    def pedir_mapa_teorico(self):
        if not self.cli_teoria.service_is_ready():
            return
        req = Trigger.Request()
        self.cli_teoria.call_async(req).add_done_callback(self.al_recibir_teoria)

    def al_recibir_teoria(self, futuro):
        try:
            res = futuro.result()
            if res.success:
                lista = json.loads(res.message)
                # TRUCO: Forzar claves a string evita bugs silenciosos de búsqueda en diccionarios
                self.paneles_teoria = {str(p['id']): p for p in lista}
                self.get_logger().info(f"Plano Teórico cargado: {len(self.paneles_teoria)} paneles listos.")
        except Exception as e:
            self.get_logger().error(f"Error cargando teoría: {e}")

    def actualizar_teoria_callback(self, msg):
        self.pedir_mapa_teorico()

    # NUEVO: Callback para actualizar dinámicamente la inclinación del gimbal
    def param_callback(self, msg):
        if len(msg.data) >= 1: 
            self.angulo_cam = float(msg.data[0])

    def calcular_vector_medido(self, p_cam, r_dron_base, p_reflex):
        # Aplicamos la inclinación DINÁMICA de la cámara respecto al chasis del dron
        r_cam_real = r_dron_base * R.from_euler('y', self.angulo_cam, degrees=False)
        
        # 1. Posición del LED en el mundo global
        p_led = p_cam + r_cam_real.apply(self.d_cam_led)
        
        # 2. Vector reflejado (Espejo -> Cámara)
        v_reflejado = p_cam - p_reflex
        v_reflejado_unitario = v_reflejado / np.linalg.norm(v_reflejado)
        
        # 3. Vector incidente (Espejo -> LED)
        v_incidente = p_led - p_reflex
        v_incidente_unitario = v_incidente / np.linalg.norm(v_incidente)
        
        # 4. Bisectriz (Normal requerida por la ley de reflexión)
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

        if not datos_filtrados:
            return

        resultados_iteracion = []

        for dato in datos_filtrados:
            id_panel_raw = dato.get("id_panel")
            if id_panel_raw is None:
                continue
            
            id_panel = str(id_panel_raw)
            
            # --- 1. OBTENER LA TEORÍA ESTRICTA ---
            panel_teo = self.paneles_teoria.get(id_panel)
            if not panel_teo:
                continue
                
            p_teo = np.array([panel_teo['x'], panel_teo['y'], panel_teo['z']])
            pitch_base = float(panel_teo['pitch'])
            yaw_base = float(panel_teo['yaw'])
            
            r_teo_original = R.from_euler('xyz', [0.0, pitch_base, yaw_base])
            n_teo_global = r_teo_original.apply([0.0, 0.0, 1.0])
            
            # --- 2. EXTRACCIÓN SEGURA DE DATOS DEL SENSOR ---
            try:
                # Forzamos a float por si llega texto, y comprobamos integridad
                p_cam = np.array(dato["dron"]["pos"], dtype=float)
                quat = np.array(dato["dron"]["quat"], dtype=float)
                p_rebote_local = np.array(dato["rebote_local"], dtype=float)
                
                # ESCUDO ANTI-FALLOS: Ignorar NaNs o cuaterniones inválidos [0,0,0,0]
                if np.any(np.isnan(p_cam)) or np.any(np.isnan(quat)) or np.any(np.isnan(p_rebote_local)):
                    self.get_logger().warn(f"Dato ignorado: Contiene valores NaN en panel {id_panel}", throttle_duration_sec=2.0)
                    continue
                    
                if np.linalg.norm(quat) < 1e-6:
                    self.get_logger().warn(f"Dato ignorado: Cuaternión nulo en panel {id_panel}", throttle_duration_sec=2.0)
                    continue

                r_cam = R.from_quat(quat)
                
            except (KeyError, ValueError, TypeError) as e:
                self.get_logger().warn(f"Dato malformado, error de formato: {e}", throttle_duration_sec=2.0)
                continue
            
            # --- 3. BUCLE ITERATIVO LOCAL (ALINEACIÓN VECTORIAL PURA) ---
            # Partimos de la teoría, y buscaremos la rotación que haga que el rayo encaje perfecto
            r_iter = r_teo_original
            n_meas_global = np.array([0.0, 0.0, 0.0])
            
            for _ in range(3):
                # Proyectar el píxel de la cámara en el mundo 3D
                p_reflex_global = p_teo + r_iter.apply(p_rebote_local)
                
                # Calcular hacia dónde nos dice la luz que DEBERÍA mirar el espejo
                n_meas_global = self.calcular_vector_medido(p_cam, r_cam, p_reflex_global)
                
                # Hacia dónde ESTÁ mirando nuestro modelo matemático ahora mismo
                n_curr_global = r_iter.apply([0.0, 0.0, 1.0])
                
                # Producto cruzado para encontrar el eje de rotación más corto entre ambos
                cross_prod = np.cross(n_curr_global, n_meas_global)
                norm_cross = np.linalg.norm(cross_prod)
                
                if norm_cross > 1e-8:
                    axis = cross_prod / norm_cross
                    dot_prod = np.dot(n_curr_global, n_meas_global)
                    
                    # Acotador de seguridad contra errores de precisión flotante en Python
                    dot_prod = max(-1.0, min(1.0, dot_prod))
                    angle = math.acos(dot_prod)
                    
                    # Generamos la corrección y la aplicamos a nuestra iteración
                    r_corr = R.from_rotvec(axis * angle)
                    r_iter = r_corr * r_iter  # Pre-multiplicación (Aplica rotación global)
            
            # --- 4. EXTRACCIÓN DEL ERROR FINAL DE ESTE DATO ---
            # r_iter es ahora la orientación "perfecta" para este punto de luz concreto.
            n_final_global = r_iter.apply([0.0, 0.0, 1.0])
            
            # Lo llevamos al sistema local de la teoría original para ver cuánto se ha movido
            n_final_ccs = r_teo_original.inv().apply(n_final_global)
            
            # Proyectamos en los planos YZ y XZ para sacar el offset angular exacto
            # NOTA: Mantenemos el signo menos en la X que añadimos para la regla de la mano derecha
            error_rotX_rad = -math.atan2(n_final_ccs[1], n_final_ccs[2])
            error_rotY_rad = math.atan2(n_final_ccs[0], n_final_ccs[2])
            
            rotX_mrad = error_rotX_rad * 1000.0
            rotY_mrad = error_rotY_rad * 1000.0
            
            # --- 5. HISTORIAL Y MEDIA MÓVIL ---
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
                "normal_medida": n_meas_global.tolist()
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
