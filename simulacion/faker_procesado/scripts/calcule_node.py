#!/usr/bin/env python3

import sys
import os
import threading
import subprocess
import numpy as np
import math
import json
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from std_msgs.msg import Float64MultiArray, String

def get_quaternion_from_euler(roll, pitch, yaw):
    r = R.from_euler('xyz', [roll, pitch, yaw], degrees=False)
    return r.as_quat()

class CalculadoraNode(Node):
    def __init__(self, nombre_mundo="prueba1", modelo_dron="x500"):
        super().__init__('calculadora_node')
        
        # Parámetros de cámara por defecto
        self.angulo_cam = 0.785
        self.dist_foc_cam = 0.0
        self.distor_cam = 0.0
        
        self.nombre_mundo = nombre_mundo
        self.modelo_dron = modelo_dron
        
        # --- SUBSCRIPCIONES ---
        self.sub_paneles_json = self.create_subscription(
            String, 
            '/sim_data/paneles_info', 
            self.paneles_json_callback, 
            10
        )
        
        self.sub_param_control = self.create_subscription(
            Float64MultiArray, 
            '/parametros_control', 
            self.param_control_callback, 
            qos_profile_sensor_data
        )
        
        self.sub_sim_activa = self.create_subscription(
            String,
            '/sim_status/sim_activa', 
            self.sim_activa_callback,
            10
        )
        
        # --- PUBLICADORES GEOMÉTRICOS ---
        self.pub_paneles = self.create_publisher(PoseArray, '/datos/paneles', 10)
        self.pub_dron = self.create_publisher(PoseStamped, '/datos/dron', 10)
        self.pub_camara = self.create_publisher(PoseStamped, '/datos/camara', 10)
        self.pub_luz = self.create_publisher(PoseStamped, '/datos/luz', 10)
        self.pub_rebotes = self.create_publisher(PoseArray, '/datos/rebotes', 10)
        self.pub_reflejos = self.create_publisher(PoseArray, '/datos/reflejos', 10)
        
        self.pub_datos_consolidados = self.create_publisher(String, '/inspeccion/datos_crudos', 10)

        # Variables de estado
        self.param_mat = []
        self.msg_paneles = PoseArray()
        self.msg_paneles.header.frame_id = "world"
        
        # Control del proceso de Gazebo
        self.proceso_gz = None 
        self.hilo_gz = None
        
        # Lanzamos el espía inicial
        self.lanzar_espia_gazebo()

        self.get_logger().info(f"Calculadora Node iniciado. Esperando mapa en el mundo '{self.nombre_mundo}'...")

    # ==========================================
    # CALLBACKS DE RECEPCIÓN DE DATOS
    # ==========================================
    def paneles_json_callback(self, msg):
        try:
            array_paneles = json.loads(msg.data)
            
            # Si el array está vacío (se ha vaciado el mundo), limpiamos
            if not array_paneles:
                self.param_mat = []
                self.msg_paneles = PoseArray()
                self.msg_paneles.header.frame_id = "world"
                self.get_logger().info("Mapa vaciado en Calculadora.")
                self.publicar_paneles() 
                return

            # Si es el mismo mapa que ya tenemos cargado, ahorramos CPU
            if len(array_paneles) == len(self.param_mat):
                return

            self.get_logger().info(f"Recibidos {len(array_paneles)} paneles. Procesando matemáticas...")
            
            self.param_mat = []
            self.msg_paneles = PoseArray()
            self.msg_paneles.header.frame_id = "world"
            
            for panel in array_paneles:
                id_panel = panel['id']
            
                # Extraemos posición y ángulos
                x, y, z = panel['x'], panel['y'], panel['z']
                pitch, yaw = panel['pitch'], panel['yaw']
                
                # Extraemos dimensiones (usamos fallback a las medidas estándar si fallara el JSON)
                width = panel.get('width_x', 10.421)
                length = panel.get('length_y', 11.415)
                
                # Asumimos roll = 0.0
                q = get_quaternion_from_euler(0.0, float(pitch), float(yaw))
                p = [float(x), float(y), float(z)]
                
                # Guardamos el ID, posición, orientación Y LAS DIMENSIONES
                self.param_mat.append([id_panel, p, q, width, length])
                
                # Mensaje visual de PoseArray
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = p[0], p[1], p[2]
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q[0], q[1], q[2], q[3]
                self.msg_paneles.poses.append(pose)
            
            # Publicación inmediata de los paneles procesados
            self.publicar_paneles()
            
        except Exception as e:
            self.get_logger().error(f"Error procesando JSON de paneles: {e}")

    def param_control_callback(self, msg):
        if len(msg.data) >= 3:
            self.angulo_cam = msg.data[0]
            self.dist_foc_cam = msg.data[1]
            self.distor_cam = msg.data[2]
    
    def sim_activa_callback(self, msg):
        try:
            datos = json.loads(msg.data)
            nuevo_mundo = datos.get("mundo")
            nuevo_dron = datos.get("dron")
            
            if nuevo_mundo != self.nombre_mundo or nuevo_dron != self.modelo_dron:
                self.get_logger().info(f"¡Nueva simulación detectada! Mundo: '{nuevo_mundo}', Dron: '{nuevo_dron}'")
                
                self.nombre_mundo = nuevo_mundo
                self.modelo_dron = nuevo_dron
                
                self.lanzar_espia_gazebo() 
                
        except json.JSONDecodeError:
            pass
            
    def publicar_paneles(self):
        self.msg_paneles.header.stamp = self.get_clock().now().to_msg()
        self.pub_paneles.publish(self.msg_paneles)

    # ==========================================
    # LÓGICA DE CONEXIÓN CON GAZEBO
    # ==========================================
    def lanzar_espia_gazebo(self):
        if self.proceso_gz is not None:
            self.get_logger().info(f"Cerrando escucha del mundo anterior...")
            self.proceso_gz.terminate()
            self.proceso_gz.wait()

        self.hilo_gz = threading.Thread(
            target=self.escuchar_gazebo_nativo, 
            args=(self.nombre_mundo, self.modelo_dron)
        )
        self.hilo_gz.daemon = True 
        self.hilo_gz.start()

    def escuchar_gazebo_nativo(self, nombre_mundo, modelo_dron):
        comando = ["gz", "topic", "-e", "-t", f"/world/{nombre_mundo}/pose/info"]
        
        self.proceso_gz = subprocess.Popen(comando, stdout=subprocess.PIPE, text=True, bufsize=1)
        
        leyendo_dron = False
        leyendo_posicion = False
        leyendo_orientacion = False
        x_gz = y_gz = z_gz = 0.0
        qw_gz = 1.0; qx_gz = qy_gz = qz_gz = 0.0
        
        for linea in iter(self.proceso_gz.stdout.readline, ''):
            linea = linea.strip()
            
            if f'name: "{modelo_dron}_0"' in linea:
                leyendo_dron = True
                continue
            elif 'name: ' in linea and leyendo_dron:
                leyendo_dron = False
                
                if len(self.param_mat) > 0:
                    self.procesar_geometria(x_gz, y_gz, z_gz, qw_gz, qx_gz, qy_gz, qz_gz)
                continue
            
            if leyendo_dron:
                if 'position {' in linea:
                    leyendo_posicion = True; leyendo_orientacion = False
                elif 'orientation {' in linea:
                    leyendo_orientacion = True; leyendo_posicion = False
                elif '}' in linea: pass 
                elif leyendo_posicion:
                    if linea.startswith('x:'): x_gz = float(linea.split(':')[1])
                    elif linea.startswith('y:'): y_gz = float(linea.split(':')[1])
                    elif linea.startswith('z:'): z_gz = float(linea.split(':')[1])
                elif leyendo_orientacion:
                    if linea.startswith('x:'): qx_gz = float(linea.split(':')[1])
                    elif linea.startswith('y:'): qy_gz = float(linea.split(':')[1])
                    elif linea.startswith('z:'): qz_gz = float(linea.split(':')[1])
                    elif linea.startswith('w:'): qw_gz = float(linea.split(':')[1])

    # ==========================================
    # CÁLCULOS MATEMÁTICOS Y ÓPTICOS
    # ==========================================
    def procesar_geometria(self, x_gz, y_gz, z_gz, qw_gz, qx_gz, qy_gz, qz_gz):
        stamp = self.get_clock().now().to_msg()
        
        # 1. Dron
        msg_dron = PoseStamped()
        msg_dron.header.frame_id = "world"
        msg_dron.header.stamp = stamp
        msg_dron.pose.position.x, msg_dron.pose.position.y, msg_dron.pose.position.z = x_gz, y_gz, z_gz
        msg_dron.pose.orientation.w, msg_dron.pose.orientation.x, msg_dron.pose.orientation.y, msg_dron.pose.orientation.z = qw_gz, qx_gz, qy_gz, qz_gz
        self.pub_dron.publish(msg_dron)
        
        # 2. Luz
        pos_cam = np.array([x_gz, y_gz, z_gz])
        rot_dron = R.from_quat([qx_gz, qy_gz, qz_gz, qw_gz]) * R.from_euler('y', self.angulo_cam, degrees=False)
        
        # camara
        msg_cam = PoseStamped()
        msg_cam.header.frame_id = "world"
        msg_cam.header.stamp = stamp
        msg_cam.pose.position.x, msg_cam.pose.position.y, msg_cam.pose.position.z = x_gz, y_gz, z_gz
        msg_cam.pose.orientation.x, msg_cam.pose.orientation.y, msg_cam.pose.orientation.z, msg_cam.pose.orientation.w = rot_dron.as_quat()
        self.pub_camara.publish(msg_cam)
        
        # cont. luz
        pos_src = pos_cam + rot_dron.apply(np.array([0.0, 0.0, -0.6]))
        
        msg_luz = PoseStamped()
        msg_luz.header.frame_id = "world"
        msg_luz.header.stamp = stamp
        msg_luz.pose.position.x, msg_luz.pose.position.y, msg_luz.pose.position.z = pos_src[0], pos_src[1], pos_src[2]
        self.pub_luz.publish(msg_luz)
        
        # 3. Calcular Rebotes
        msg_rebotes = PoseArray()
        msg_rebotes.header.frame_id = "world"
        msg_rebotes.header.stamp = stamp
        
        msg_reflejos = PoseArray()
        msg_reflejos.header.frame_id = "world"
        msg_reflejos.header.stamp = stamp
        
        lista_datos_consolidados = []
        
        for param in self.param_mat:
            id_panel = param[0]
            pos_panel = np.array(param[1])
            rot_panel = R.from_quat(param[2])
            width = float(param[3])
            length = float(param[4])
            
            rot_panel_inv = rot_panel.inv()
            
            cam_local = rot_panel_inv.apply(pos_cam - pos_panel)
            src_local = rot_panel_inv.apply(pos_src - pos_panel)
            
            if cam_local[2] <= 0 or src_local[2] <= 0:
                continue
            
            ref_local = np.array([src_local[0], src_local[1], -src_local[2]])
            
            denominador = cam_local[2] - ref_local[2]
            if abs(denominador) < 1e-6: continue # Evitamos divisiones por cero
            
            t = -ref_local[2] / denominador
            I_local = ref_local + t * (cam_local - ref_local)
            
            if abs(I_local[0]) <= (width / 2.0) and abs(I_local[1]) <= (length / 2.0): 
                I_world = pos_panel + rot_panel.apply(I_local)
                ref_world = pos_panel + rot_panel.apply(ref_local)
                
                # Estos se siguen publicando en global para Rviz
                pose_rebote = Pose()
                pose_rebote.position.x, pose_rebote.position.y, pose_rebote.position.z = I_world[0], I_world[1], I_world[2]
                msg_rebotes.poses.append(pose_rebote)
                
                pose_reflejo = Pose()
                pose_reflejo.position.x, pose_reflejo.position.y, pose_reflejo.position.z = ref_world[0], ref_world[1], ref_world[2]
                msg_reflejos.poses.append(pose_reflejo)
                
                normal_teorica = rot_panel.apply([0.0, 0.0, 1.0])
                
                # --- AQUÍ ESTÁ EL CAMBIO CLAVE ---
                dato_impacto = {
                    "id_panel": id_panel,
                    "rebote_local": [float(I_local[0]), float(I_local[1]), float(I_local[2])], # ¡MANDAMOS EL LOCAL!
                    "normal_teorica": [float(normal_teorica[0]), float(normal_teorica[1]), float(normal_teorica[2])],
                    "pose_panel": { # MANDAMOS LA POSE DEL PANEL
                        "pos": [float(pos_panel[0]), float(pos_panel[1]), float(pos_panel[2])],
                        "quat": [float(rot_panel.as_quat()[0]), float(rot_panel.as_quat()[1]), float(rot_panel.as_quat()[2]), float(rot_panel.as_quat()[3])]
                    },
                    "dron": {
                        "pos": [float(x_gz), float(y_gz), float(z_gz)],
                        "quat": [float(qx_gz), float(qy_gz), float(qz_gz), float(qw_gz)]
                    }
                }
                lista_datos_consolidados.append(dato_impacto)
                # ---------------------------------
                
        self.pub_rebotes.publish(msg_rebotes)
        self.pub_reflejos.publish(msg_reflejos)
        
        if lista_datos_consolidados:
            msg_json = String()
            msg_json.data = json.dumps(lista_datos_consolidados)
            self.pub_datos_consolidados.publish(msg_json)

def main(args=None):
    rclpy.init(args=args)
    nodo = CalculadoraNode(nombre_mundo="prueba1", modelo_dron="x500")
    try: 
        rclpy.spin(nodo)
    except KeyboardInterrupt: 
        pass
    finally:
        if nodo.proceso_gz is not None:
            nodo.proceso_gz.terminate()
            nodo.proceso_gz.wait()
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
