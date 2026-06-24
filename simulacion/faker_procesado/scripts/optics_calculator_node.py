#!/usr/bin/env python3

import json
import subprocess
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation as R
from scipy.spatial import KDTree
from std_msgs.msg import Float64MultiArray, String
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from std_srvs.srv import Trigger

class OpticsCalculatorNode(Node):
    def __init__(self, nombre_mundo="prueba1", modelo_dron="x500"):
        super().__init__('optics_calculator_node')
        
        self.nombre_mundo = nombre_mundo
        self.modelo_dron = modelo_dron
        
        self.pos_dron = None
        self.quat_dron = None
        self.angulo_cam = 0.785   
        self.distancia_max_vision = 500.0 
        self.pose_nueva_disponible = False

        self.fov_grados = 90.0
        self.limite_coseno_fov = np.cos(np.radians(self.fov_grados / 2.0))

        # MEMORIA RAM PURA: Aquí guardamos el JSON
        self.paneles_memoria = []
        self.coords_paneles = []
        self.tamanios_paneles = {}
        self.arbol_kd = None

        self.cli_realidad = self.create_client(Trigger, 'get_panel_real')
        
        self.pub_datos_raw = self.create_publisher(String, '/inspection/raw_data', qos_profile_sensor_data)
        self.pub_rebotes_viz = self.create_publisher(PoseArray, '/data/impacts', qos_profile_sensor_data)
        self.pub_dron = self.create_publisher(PoseStamped, '/data/drone', qos_profile_sensor_data)
        self.pub_camara = self.create_publisher(PoseStamped, '/data/camera', qos_profile_sensor_data)
        self.pub_luz = self.create_publisher(PoseStamped, '/data/light', qos_profile_sensor_data)

        self.create_subscription(Float64MultiArray, '/control_param', self.param_callback, qos_profile_sensor_data)
        self.create_subscription(String, '/sim_status/panel_updates', self.panel_update_callback, 10)

        self.create_timer(0.05, self.bucle_percepcion_hz)

        self.pedir_mapa_completo()
        self.lanzar_espia_gazebo()
        self.get_logger().info("Calculadora Óptica [KD-Tree + RAM Pura] iniciada. Cero latencia.")

    def panel_update_callback(self, msg):
        self.pedir_mapa_completo()

    def pedir_mapa_completo(self):
        if not self.cli_realidad.service_is_ready(): return
        req = Trigger.Request()
        self.cli_realidad.call_async(req).add_done_callback(self.al_recibir_mapa)

    def al_recibir_mapa(self, futuro):
        try:
            res = futuro.result()
            if res.success:
                self.paneles_memoria = json.loads(res.message)
                self.nombres_paneles = [p['id'] for p in self.paneles_memoria]
                self.coords_paneles = [[p['x'], p['y'], p['z']] for p in self.paneles_memoria]
                
                if self.coords_paneles:
                    self.arbol_kd = KDTree(self.coords_paneles)
                
                self.tamanios_paneles = {p['id']: {'w': p.get('width_x', 10.4), 'l': p.get('length_y', 11.4)} for p in self.paneles_memoria}
        except Exception as e:
            self.get_logger().error(f"Error procesando metadatos: {e}")

    def param_callback(self, msg):
        if len(msg.data) >= 1: self.angulo_cam = msg.data[0]

    def bucle_percepcion_hz(self):
        if not self.pose_nueva_disponible or self.pos_dron is None or self.arbol_kd is None:
            return
            
        self.pose_nueva_disponible = False 
        stamp = self.get_clock().now().to_msg()
        
        rot_dron = R.from_quat(self.quat_dron)
        rot_cam = rot_dron * R.from_euler('y', self.angulo_cam)
        eje_optico = rot_cam.apply([1, 0, 0])
        
        msg_dron = PoseStamped()
        msg_dron.header.frame_id, msg_dron.header.stamp = "world", stamp
        msg_dron.pose.position.x, msg_dron.pose.position.y, msg_dron.pose.position.z = self.pos_dron
        msg_dron.pose.orientation.x, msg_dron.pose.orientation.y, msg_dron.pose.orientation.z, msg_dron.pose.orientation.w = self.quat_dron
        self.pub_dron.publish(msg_dron)

        msg_cam = PoseStamped()
        msg_cam.header.frame_id, msg_cam.header.stamp = "world", stamp
        msg_cam.pose.position = msg_dron.pose.position
        msg_cam.pose.orientation.x, msg_cam.pose.orientation.y, msg_cam.pose.orientation.z, msg_cam.pose.orientation.w = rot_cam.as_quat()
        self.pub_camara.publish(msg_cam)

        pos_luz = self.pos_dron + rot_cam.apply([0, 0, -0.6])
        msg_luz = PoseStamped()
        msg_luz.header.frame_id, msg_luz.header.stamp = "world", stamp
        msg_luz.pose.position.x, msg_luz.pose.position.y, msg_luz.pose.position.z = pos_luz
        self.pub_luz.publish(msg_luz)

        impactos = []
        msg_viz = PoseArray()
        msg_viz.header.frame_id, msg_viz.header.stamp = "world", stamp

        indices_cercanos = self.arbol_kd.query_ball_point(self.pos_dron, r=self.distancia_max_vision)

        for idx in indices_cercanos:
            p = self.paneles_memoria[idx]
            id_panel = p['id']
            pos_p_global = np.array([p['x'], p['y'], p['z']])
            
            vec_dir_poste = pos_p_global - self.pos_dron
            dist = np.linalg.norm(vec_dir_poste)
            if dist < 1e-3: continue
            
            coseno_angulo = np.dot(eje_optico, vec_dir_poste / dist)
            if coseno_angulo < self.limite_coseno_fov:
                continue 
                
            tam = self.tamanios_paneles.get(id_panel, {'w': 10.4, 'l': 11.4})
            w_f = tam['w'] / 5.0
            l_f = tam['l'] / 5.0

            # TU CINEMÁTICA ORIGINAL (Síncrona y robusta)
            rot_yaw = R.from_euler('z', p.get('yaw', 0.0))
            rot_pitch = R.from_euler('y', p.get('pitch', 0.0))
            r_p_global = rot_yaw * rot_pitch

            if 'facetas' in p:
                for f in p['facetas']:
                    id_faceta_raw = f"{id_panel}_f{f['id']}" if not f['id'].startswith(id_panel) else f['id']
                    
                    offset_local = np.array(f.get('offset', [0.0, 0.0, 0.0]))
                    pos_f = pos_p_global + r_p_global.apply(offset_local)
                    
                    rot_canting = R.from_euler('xy', [f.get('cant_roll', 0.0), f.get('cant_pitch', 0.0)])
                    rot_f = r_p_global * rot_canting

                    inv_rot_f = rot_f.inv()
                    cam_loc = inv_rot_f.apply(self.pos_dron - pos_f)
                    luz_loc = inv_rot_f.apply(pos_luz - pos_f)

                    if cam_loc[2] <= 0: continue 

                    ref_loc = np.array([luz_loc[0], luz_loc[1], -luz_loc[2]])
                    denom = cam_loc[2] - ref_loc[2]
                    if abs(denom) < 1e-6: continue
                    
                    i_loc = ref_loc + (-ref_loc[2] / denom) * (cam_loc - ref_loc)

                    if abs(i_loc[0]) <= (w_f/2) and abs(i_loc[1]) <= (l_f/2):
                        i_world = pos_f + rot_f.apply(i_loc)
                        impactos.append({
                            "id_panel": id_faceta_raw,
                            "rebote_local": i_loc.tolist(),
                            "rebote_world_debug": i_world.tolist(),
                            "dron": {
                                "pos": self.pos_dron.tolist(),
                                "quat": self.quat_dron.tolist()
                            }
                        })
                        pv = Pose()
                        pv.position.x, pv.position.y, pv.position.z = i_world
                        msg_viz.poses.append(pv)
            else:
                pass # Lógica panel simple si aplicara

        if impactos:
            self.pub_datos_raw.publish(String(data=json.dumps(impactos)))
            self.pub_rebotes_viz.publish(msg_viz)

    def lanzar_espia_gazebo(self):
        self.hilo_gz = threading.Thread(target=self.escuchar_gazebo, daemon=True)
        self.hilo_gz.start()

    def escuchar_gazebo(self):
        # FILTRO LINUX: Extrae solo el dron desde el tópico global
        comando = f"gz topic -e -t /world/{self.nombre_mundo}/pose/info | grep --line-buffered -A 12 'name: \"{self.modelo_dron}_0\"'"
        proc = subprocess.Popen(comando, stdout=subprocess.PIPE, text=True, shell=True)
        
        in_pos, in_ori = False, False
        c_p = [0.0, 0.0, 0.0]
        c_q = [0.0, 0.0, 0.0, 1.0]

        for linea in iter(proc.stdout.readline, ''):
            linea = linea.strip()
            if "position" in linea: in_pos, in_ori = True, False; continue
            if "orientation" in linea: in_pos, in_ori = False, True; continue
            
            if in_pos:
                if "x:" in linea: c_p[0] = float(linea.split(":")[1])
                if "y:" in linea: c_p[1] = float(linea.split(":")[1])
                if "z:" in linea: c_p[2] = float(linea.split(":")[1])
            if in_ori:
                if "x:" in linea: c_q[0] = float(linea.split(":")[1])
                if "y:" in linea: c_q[1] = float(linea.split(":")[1])
                if "z:" in linea: c_q[2] = float(linea.split(":")[1])
                if "w:" in linea: c_q[3] = float(linea.split(":")[1])

            if "}" in linea and in_ori:
                self.pos_dron, self.quat_dron = np.array(c_p), np.array(c_q)
                self.pose_nueva_disponible = True  
                in_pos = in_ori = False

def main(args=None):
    rclpy.init(args=args)
    nodo = OpticsCalculatorNode()
    try: rclpy.spin(nodo)
    except KeyboardInterrupt: pass
    finally:
        if rclpy.ok(): nodo.destroy_node(); rclpy.shutdown()

if __name__ == "__main__": main()
