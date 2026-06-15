#!/usr/bin/env python3

import json
import subprocess
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Float64MultiArray, String
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from std_srvs.srv import Trigger 

class OpticsCalculatorNode(Node):
    def __init__(self, nombre_mundo="prueba1", modelo_dron="x500"):
        super().__init__('optics_calculator_node')
        
        self.nombre_mundo = nombre_mundo
        self.modelo_dron = modelo_dron
        
        # --- ESTADO INTERNO ---
        self.paneles_reales = {} 
        self.pos_dron = None
        self.quat_dron = None
        self.angulo_cam = 0.785   
        self.distancia_max_vision = 500.0 
        
        # Bandera para sincronizar el hilo lector y el hilo calculador
        self.pose_nueva_disponible = False

        self.cli_realidad = self.create_client(Trigger, 'get_panel_real')
        
        # Usamos qos_profile_sensor_data para evitar colas (Drop old messages)
        self.pub_datos_raw = self.create_publisher(String, '/inspection/raw_data', qos_profile_sensor_data)
        self.pub_rebotes_viz = self.create_publisher(PoseArray, '/data/impacts', qos_profile_sensor_data)
        self.pub_panels_viz = self.create_publisher(PoseArray, '/data/panels', 10)
        self.pub_dron = self.create_publisher(PoseStamped, '/data/drone', qos_profile_sensor_data)
        self.pub_camara = self.create_publisher(PoseStamped, '/data/camera', qos_profile_sensor_data)
        self.pub_luz = self.create_publisher(PoseStamped, '/data/light', qos_profile_sensor_data)

        self.create_subscription(Float64MultiArray, '/control_param', self.param_callback, qos_profile_sensor_data)
        self.create_subscription(String, '/sim_status/panel_updates', self.panel_update_callback, 10)

        # --- NUEVO: BUCLE DE PERCEPCIÓN DESACOPLADO A 20 FPS ---
        self.create_timer(0.05, self.bucle_percepcion_hz)

        self.pedir_mapa_completo()
        self.lanzar_espia_gazebo()
        
        self.get_logger().info("Calculadora Óptica Iniciada [MODO ANTI-LAG ACTIVO].")

    def pedir_mapa_completo(self):
        if not self.cli_realidad.service_is_ready(): return
        req = Trigger.Request()
        self.cli_realidad.call_async(req).add_done_callback(self.al_recibir_mapa)

    def al_recibir_mapa(self, futuro):
        try:
            res = futuro.result()
            if res.success:
                lista = json.loads(res.message)
                self.paneles_reales = {p['id']: p for p in lista}
                self.publicar_estatica_paneles()
        except Exception as e:
            self.get_logger().error(f"Error servicio: {e}")

    def publicar_estatica_paneles(self):
        msg = PoseArray()
        msg.header.frame_id = "world"
        msg.header.stamp = self.get_clock().now().to_msg()
        
        for p in self.paneles_reales.values():
            if 'facetas' in p:
                r_track = R.from_euler('xyz', [0.0, p['pitch'], p['yaw']])
                p_centro = np.array([p['x'], p['y'], p['z']])
                for f in p['facetas']:
                    pose = Pose()
                    pose.position.x, pose.position.y, pose.position.z = p_centro + r_track.apply(f['offset'])
                    q = (r_track * R.from_euler('xyz', [0.0, f['cant_pitch'], f['cant_yaw']])).as_quat()
                    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
                    msg.poses.append(pose)
            else:
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = p['x'], p['y'], p['z']
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = R.from_euler('xyz', [0.0, p['pitch'], p['yaw']]).as_quat()
                msg.poses.append(pose)
                
        self.pub_panels_viz.publish(msg)

    def panel_update_callback(self, msg):
        self.pedir_mapa_completo()

    def param_callback(self, msg):
        if len(msg.data) >= 1: self.angulo_cam = msg.data[0]

    # --- NUEVO: ESTA FUNCIÓN SE EJECUTA SIEMPRE A 20 FPS EXACTOS ---
    def bucle_percepcion_hz(self):
        if not self.pose_nueva_disponible or self.pos_dron is None or not self.paneles_reales:
            return
            
        self.pose_nueva_disponible = False # Reseteamos la bandera
        stamp = self.get_clock().now().to_msg()
        
        # (El resto del código matemático es exactamente el mismo de antes)
        rot_dron = R.from_quat(self.quat_dron)
        rot_cam = rot_dron * R.from_euler('y', self.angulo_cam)
        
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

        targets_opticos = []
        for p in self.paneles_reales.values():
            if 'facetas' in p:
                r_track = R.from_euler('xyz', [0.0, p['pitch'], p['yaw']])
                p_centro = np.array([p['x'], p['y'], p['z']])
                w_f = p.get('width_x', 10.4) / 5.0
                l_f = p.get('length_y', 11.4) / 5.0
                for f in p['facetas']:
                    targets_opticos.append({
                        'id': f['id'], 
                        'pos': p_centro + r_track.apply(f['offset']), 
                        'rot': r_track * R.from_euler('xyz', [0.0, f['cant_pitch'], f['cant_yaw']]), 
                        'w': w_f, 'l': l_f
                    })
            else:
                targets_opticos.append({
                    'id': p['id'], 'pos': np.array([p['x'], p['y'], p['z']]), 
                    'rot': R.from_euler('xyz', [0.0, p['pitch'], p['yaw']]), 
                    'w': p.get('width_x', 10.4), 'l': p.get('length_y', 11.4)
                })

        impactos = []
        msg_viz = PoseArray()
        msg_viz.header.frame_id, msg_viz.header.stamp = "world", stamp
        eje_optico = rot_cam.apply([1, 0, 0])

        for t in targets_opticos:
            vec_dir = (t['pos'] - self.pos_dron)
            dist = np.linalg.norm(vec_dir)
            if dist > self.distancia_max_vision or np.dot(eje_optico, vec_dir/dist) < 0.1: continue

            inv_rot_t = t['rot'].inv()
            cam_loc = inv_rot_t.apply(self.pos_dron - t['pos'])
            luz_loc = inv_rot_t.apply(pos_luz - t['pos'])

            if cam_loc[2] <= 0: continue 

            ref_loc = np.array([luz_loc[0], luz_loc[1], -luz_loc[2]])
            denom = cam_loc[2] - ref_loc[2]
            if abs(denom) < 1e-6: continue
            
            i_loc = ref_loc + (-ref_loc[2] / denom) * (cam_loc - ref_loc)

            if abs(i_loc[0]) <= (t['w']/2) and abs(i_loc[1]) <= (t['l']/2):
                i_world = t['pos'] + t['rot'].apply(i_loc)
                impactos.append({
                    "id_panel": t['id'],
                    "rebote_local": i_loc.tolist(),
                    "rebote_world_debug": i_world.tolist(),
                    "pose_panel": {"pos": t['pos'].tolist(), "quat": t['rot'].as_quat().tolist()},
                    "dron": {"pos": self.pos_dron.tolist(), "quat": self.quat_dron.tolist()}
                })
                pv = Pose()
                pv.position.x, pv.position.y, pv.position.z = i_world
                msg_viz.poses.append(pv)

        if impactos:
            self.pub_datos_raw.publish(String(data=json.dumps(impactos)))
            self.pub_rebotes_viz.publish(msg_viz)

    def lanzar_espia_gazebo(self):
        self.hilo_gz = threading.Thread(target=self.escuchar_gazebo, daemon=True)
        self.hilo_gz.start()

    # --- LECTURA PURA Y RÁPIDA DE GAZEBO ---
    def escuchar_gazebo(self):
        comando = ["gz", "topic", "-e", "-t", f"/world/{self.nombre_mundo}/pose/info"]
        proc = subprocess.Popen(comando, stdout=subprocess.PIPE, text=True)
        target = f'name: "{self.modelo_dron}_0"'
        
        leyendo_dron, in_pos, in_ori = False, False, False
        c_p = [0.0, 0.0, 0.0]
        c_q = [0.0, 0.0, 0.0, 1.0]

        for linea in iter(proc.stdout.readline, ''):
            linea = linea.strip()
            if target in linea: leyendo_dron = True; continue
            if not leyendo_dron: continue

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
                self.pose_nueva_disponible = True  # ¡Avisamos al bucle de arriba de que hay datos frescos!
                leyendo_dron = in_pos = in_ori = False

def main(args=None):
    rclpy.init(args=args)
    nodo = OpticsCalculatorNode()
    try: rclpy.spin(nodo)
    except KeyboardInterrupt: pass
    finally:
        if rclpy.ok():
            nodo.destroy_node()
            rclpy.shutdown()

if __name__ == "__main__": main()
