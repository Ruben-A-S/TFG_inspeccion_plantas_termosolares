import sys
import os
import threading
import subprocess
import numpy as np
import math
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Point, Quaternion
from std_msgs.msg import Float64MultiArray

def get_quaternion_from_euler(roll, pitch, yaw):
    # Usamos Scipy que es a prueba de balas para evitar fallos de orden de ejes
    r = R.from_euler('xyz', [roll, pitch, yaw], degrees=False)
    return r.as_quat()

class CalculadoraNode(Node):
    def __init__(self, nombre_mundo, archivo_txt, modelo_dron):
        super().__init__('calculadora_node')
        
        self.angulo_cam = 0.785
        self.dist_foc_cam = 0.0
        self.distor_cam = 0.0
        
        self.sub_param_control = self.create_subscription(
            Float64MultiArray, 
            '/parametros_control', 
            self.param_control_callback, 
            qos_profile_sensor_data
        )
        
        # Publicadores de datos geométricos
        self.pub_paneles = self.create_publisher(PoseArray, '/datos/paneles', 10)
        self.pub_dron = self.create_publisher(PoseStamped, '/datos/dron', 10)
        self.pub_camara = self.create_publisher(PoseStamped, '/datos/camara', 10)
        self.pub_luz = self.create_publisher(PoseStamped, '/datos/luz', 10)
        self.pub_rebotes = self.create_publisher(PoseArray, '/datos/rebotes', 10)
        self.pub_reflejos = self.create_publisher(PoseArray, '/datos/reflejos', 10)

        self.param_mat = []
        self.msg_paneles = PoseArray()
        self.msg_paneles.header.frame_id = "world"

        self.cargar_campo_solar(archivo_txt)
        
        # Publicamos los paneles periódicamente
        self.timer = self.create_timer(2.0, self.publicar_paneles)

        # Hilo de Gazebo
        self.hilo_gz = threading.Thread(
            target=self.escuchar_gazebo_nativo, 
            args=(nombre_mundo, archivo_txt, modelo_dron)
        )
        self.hilo_gz.daemon = True 
        self.hilo_gz.start()
    
    def param_control_callback(self, msg):
        if len(msg.data) >= 3:
            self.angulo_cam = msg.data[0]
            self.dist_foc_cam = msg.data[1]
            self.distor_cam = msg.data[2]
    
    def publicar_paneles(self):
        self.msg_paneles.header.stamp = self.get_clock().now().to_msg()
        self.pub_paneles.publish(self.msg_paneles)

    def escuchar_gazebo_nativo(self, nombre_mundo, archivo_txt, modelo_dron):
        comando = ["gz", "topic", "-e", "-t", f"/world/{nombre_mundo}/pose/info"]
        proceso = subprocess.Popen(comando, stdout=subprocess.PIPE, text=True, bufsize=1)
        
        leyendo_dron = False
        leyendo_posicion = False
        leyendo_orientacion = False
        x_gz = y_gz = z_gz = 0.0
        qw_gz = 1.0; qx_gz = qy_gz = qz_gz = 0.0
        
        for linea in iter(proceso.stdout.readline, ''):
            linea = linea.strip()
            
            # print(f"Gazebo dice: {linea}") # Chivato
            
            if f'name: "{modelo_dron}_0"' in linea:
                leyendo_dron = True
                continue
            elif 'name: ' in linea and leyendo_dron:
                leyendo_dron = False
                # print(f"{x_gz}, {y_gz}, {z_gz}, {qw_gz}, {qx_gz}, {qy_gz}, {qz_gz}")
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
        
        for param in self.param_mat:
            pos_panel = np.array(param[0])
            rot_panel = R.from_quat(param[1])
            rot_panel_inv = rot_panel.inv()
            
            cam_local = rot_panel_inv.apply(pos_cam - pos_panel)
            src_local = rot_panel_inv.apply(pos_src - pos_panel)
            
            if cam_local[2] <= 0 or src_local[2] <= 0:
                continue
            
            ref_local = np.array([src_local[0], src_local[1], -src_local[2]])
            t = -ref_local[2] / (cam_local[2] - ref_local[2])
            I_local = ref_local + t * (cam_local - ref_local)
            
            if abs(I_local[0]) <= 1.5 and abs(I_local[1]) <= 1.0:  # Aqui se debe ajustar el ancho
                I_world = pos_panel + rot_panel.apply(I_local)
                ref_world = pos_panel + rot_panel.apply(ref_local)
                
                # Guardamos el punto de corte
                pose_rebote = Pose()
                pose_rebote.position.x, pose_rebote.position.y, pose_rebote.position.z = I_world[0], I_world[1], I_world[2]
                msg_rebotes.poses.append(pose_rebote)
                
                # Guardamos el punto de reflejo virtual
                pose_reflejo = Pose()
                pose_reflejo.position.x, pose_reflejo.position.y, pose_reflejo.position.z = ref_world[0], ref_world[1], ref_world[2]
                msg_reflejos.poses.append(pose_reflejo)
                
        self.pub_rebotes.publish(msg_rebotes)
        self.pub_reflejos.publish(msg_reflejos)

    def cargar_campo_solar(self, archivo_txt):
        print(f"Leyendo mapa desde: {archivo_txt}")
        with open(archivo_txt, 'r') as file:
            lineas = file.readlines()
        
        for linea in lineas:
            linea_limpia = linea.strip()
            if not linea_limpia or linea_limpia.startswith('#'): continue
            parametros = linea_limpia.split()
            if len(parametros) != 6: continue
            
            nombre, x, y, z, pitch, yaw = parametros
            q = get_quaternion_from_euler(0.0, float(pitch), float(yaw))
            p = [float(x), float(y), float(z)]
            self.param_mat.append([p, q])
            
            # Guardar en el mensaje PoseArray
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = p[0], p[1], p[2]
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q[0], q[1], q[2], q[3]
            self.msg_paneles.poses.append(pose)

def main(args=None):
    if len(sys.argv) != 4: 
        print("Error: Número de argumentos incorrecto. Uso: python3 calcule_node.py <mundo> <mapa.txt> <dron>")
        sys.exit(1)
        
    if not os.path.isfile(sys.argv[2]):
        print(f"Error: No se encuentra el archivo de mapa '{sys.argv[2]}'")
        sys.exit(1)
        
    rclpy.init(args=args)
    nodo = CalculadoraNode(sys.argv[1], sys.argv[2], sys.argv[3])
    try: 
        rclpy.spin(nodo)
    except KeyboardInterrupt: 
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
