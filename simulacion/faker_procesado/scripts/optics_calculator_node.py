#!/usr/bin/env python3

import json
import subprocess
import threading

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Float64MultiArray, String


def get_quaternion_from_euler(roll, pitch, yaw):
    """Convierte ángulos de Euler a cuaterniones usando SciPy."""
    r = R.from_euler('xyz', [roll, pitch, yaw], degrees=False)
    return r.as_quat()


class OpticsCalculatorNode(Node):
    """
    Nodo encargado del cálculo óptico y geométrico de la simulación.
    
    Espía la posición del dron directamente desde Gazebo, calcula los rebotes
    y reflejos teóricos de la luz sobre los helióstatos y publica los datos
    crudos para el procesamiento de visión.
    """

    def __init__(self, nombre_mundo="prueba1", modelo_dron="x500"):
        super().__init__('optics_calculator_node')
        
        # Parámetros de cámara por defecto
        self.angulo_cam = 0.785
        self.dist_foc_cam = 0.0
        self.distor_cam = 0.0
        
        self.nombre_mundo = nombre_mundo
        self.modelo_dron = modelo_dron
        
        # --- SUSCRIPCIONES ---
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
        
        self.pub_datos_consolidados = self.create_publisher(
            String, '/inspeccion/datos_crudos', 10
        )

        # Variables de estado
        self.param_mat = []
        self.msg_paneles = PoseArray()
        self.msg_paneles.header.frame_id = "world"
        
        # Control del proceso de Gazebo
        self.proceso_gz = None 
        self.hilo_gz = None
        
        # Lanzamos el espía inicial
        self.lanzar_espia_gazebo()

        self.get_logger().info(
            f"Optics Calculator Node iniciado. "
            f"Esperando mapa en el mundo '{self.nombre_mundo}'..."
        )

    # ==========================================
    # CALLBACKS DE RECEPCIÓN DE DATOS
    # ==========================================
    
    def paneles_json_callback(self, msg):
        """Actualiza el mapa de paneles cuando se recibe una nueva inyección."""
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

            self.get_logger().info(
                f"Recibidos {len(array_paneles)} paneles. Procesando matemáticas..."
            )
            
            self.param_mat = []
            self.msg_paneles = PoseArray()
            self.msg_paneles.header.frame_id = "world"
            
            for panel in array_paneles:
                id_panel = panel['id']
                x, y, z = panel['x'], panel['y'], panel['z']
                pitch, yaw = panel['pitch'], panel['yaw']
                
                # Extraemos dimensiones con fallback a medidas estándar
                width = panel.get('width_x', 10.421)
                length = panel.get('length_y', 11.415)
                
                # Asumimos roll = 0.0
                q = get_quaternion_from_euler(0.0, float(pitch), float(yaw))
                p = [float(x), float(y), float(z)]
                
                # Guardamos el ID, posición, orientación y dimensiones
                self.param_mat.append([id_panel, p, q, width, length])
                
                # Mensaje visual de PoseArray
                pose = Pose()
                pose.position.x = p[0]
                pose.position.y = p[1]
                pose.position.z = p[2]
                pose.orientation.x = q[0]
                pose.orientation.y = q[1]
                pose.orientation.z = q[2]
                pose.orientation.w = q[3]
                self.msg_paneles.poses.append(pose)
            
            # Publicación inmediata de los paneles procesados
            self.publicar_paneles()
            
        except Exception as e:
            self.get_logger().error(f"Error procesando JSON de paneles: {e}")

    def param_control_callback(self, msg):
        """Actualiza la inclinación y enfoque de la cámara."""
        if len(msg.data) >= 3:
            self.angulo_cam = msg.data[0]
            self.dist_foc_cam = msg.data[1]
            self.distor_cam = msg.data[2]
    
    def sim_activa_callback(self, msg):
        """Reinicia el espía de Gazebo si cambia la simulación activa."""
        try:
            datos = json.loads(msg.data)
            nuevo_mundo = datos.get("mundo")
            nuevo_dron = datos.get("dron")
            
            if nuevo_mundo != self.nombre_mundo or nuevo_dron != self.modelo_dron:
                self.get_logger().info(
                    f"¡Nueva simulación detectada! Mundo: '{nuevo_mundo}', "
                    f"Dron: '{nuevo_dron}'"
                )
                
                self.nombre_mundo = nuevo_mundo
                self.modelo_dron = nuevo_dron
                self.lanzar_espia_gazebo() 
                
        except json.JSONDecodeError:
            pass
            
    def publicar_paneles(self):
        """Actualiza la estampa de tiempo y publica el array de poses."""
        self.msg_paneles.header.stamp = self.get_clock().now().to_msg()
        self.pub_paneles.publish(self.msg_paneles)

    # ==========================================
    # LÓGICA DE CONEXIÓN CON GAZEBO
    # ==========================================
    
    def lanzar_espia_gazebo(self):
        """Lanza un subproceso para espiar la telemetría nativa de Gazebo."""
        if self.proceso_gz is not None:
            self.get_logger().info("Cerrando escucha del mundo anterior...")
            self.proceso_gz.terminate()
            self.proceso_gz.wait()

        self.hilo_gz = threading.Thread(
            target=self.escuchar_gazebo_nativo, 
            args=(self.nombre_mundo, self.modelo_dron)
        )
        self.hilo_gz.daemon = True 
        self.hilo_gz.start()

    def escuchar_gazebo_nativo(self, nombre_mundo, modelo_dron):
        """Hilo en background que parsea la salida en terminal de gz topic."""
        comando = ["gz", "topic", "-e", "-t", f"/world/{nombre_mundo}/pose/info"]
        
        self.proceso_gz = subprocess.Popen(
            comando, stdout=subprocess.PIPE, text=True, bufsize=1
        )
        
        leyendo_dron = False
        leyendo_posicion = False
        leyendo_orientacion = False
        
        x_gz = y_gz = z_gz = 0.0
        qx_gz = qy_gz = qz_gz = 0.0
        qw_gz = 1.0
        
        for linea in iter(self.proceso_gz.stdout.readline, ''):
            linea = linea.strip()
            
            if f'name: "{modelo_dron}_0"' in linea:
                leyendo_dron = True
                continue
            elif 'name: ' in linea and leyendo_dron:
                leyendo_dron = False
                
                if len(self.param_mat) > 0:
                    self.procesar_geometria(
                        x_gz, y_gz, z_gz, qw_gz, qx_gz, qy_gz, qz_gz
                    )
                continue
            
            if leyendo_dron:
                if 'position {' in linea:
                    leyendo_posicion = True
                    leyendo_orientacion = False
                elif 'orientation {' in linea:
                    leyendo_orientacion = True
                    leyendo_posicion = False
                elif '}' in linea: 
                    pass 
                elif leyendo_posicion:
                    if linea.startswith('x:'):
                        x_gz = float(linea.split(':')[1])
                    elif linea.startswith('y:'):
                        y_gz = float(linea.split(':')[1])
                    elif linea.startswith('z:'):
                        z_gz = float(linea.split(':')[1])
                elif leyendo_orientacion:
                    if linea.startswith('x:'):
                        qx_gz = float(linea.split(':')[1])
                    elif linea.startswith('y:'):
                        qy_gz = float(linea.split(':')[1])
                    elif linea.startswith('z:'):
                        qz_gz = float(linea.split(':')[1])
                    elif linea.startswith('w:'):
                        qw_gz = float(linea.split(':')[1])

    # ==========================================
    # CÁLCULOS MATEMÁTICOS Y ÓPTICOS
    # ==========================================
    
    def procesar_geometria(self, x_gz, y_gz, z_gz, qw_gz, qx_gz, qy_gz, qz_gz):
        """Realiza el raytracing inverso desde la cámara a los paneles."""
        stamp = self.get_clock().now().to_msg()
        
        # 1. Dron
        msg_dron = PoseStamped()
        msg_dron.header.frame_id = "world"
        msg_dron.header.stamp = stamp
        msg_dron.pose.position.x = x_gz
        msg_dron.pose.position.y = y_gz
        msg_dron.pose.position.z = z_gz
        msg_dron.pose.orientation.x = qx_gz
        msg_dron.pose.orientation.y = qy_gz
        msg_dron.pose.orientation.z = qz_gz
        msg_dron.pose.orientation.w = qw_gz
        self.pub_dron.publish(msg_dron)
        
        # 2. Luz y Cámara
        pos_cam = np.array([x_gz, y_gz, z_gz])
        rot_dron = (
            R.from_quat([qx_gz, qy_gz, qz_gz, qw_gz]) * 
            R.from_euler('y', self.angulo_cam, degrees=False)
        )
        
        msg_cam = PoseStamped()
        msg_cam.header.frame_id = "world"
        msg_cam.header.stamp = stamp
        msg_cam.pose.position.x = x_gz
        msg_cam.pose.position.y = y_gz
        msg_cam.pose.position.z = z_gz
        
        cam_quat = rot_dron.as_quat()
        msg_cam.pose.orientation.x = cam_quat[0]
        msg_cam.pose.orientation.y = cam_quat[1]
        msg_cam.pose.orientation.z = cam_quat[2]
        msg_cam.pose.orientation.w = cam_quat[3]
        self.pub_camara.publish(msg_cam)
        
        # Cont. luz (Offset del foco de luz)
        pos_src = pos_cam + rot_dron.apply(np.array([0.0, 0.0, -0.6]))
        
        msg_luz = PoseStamped()
        msg_luz.header.frame_id = "world"
        msg_luz.header.stamp = stamp
        msg_luz.pose.position.x = pos_src[0]
        msg_luz.pose.position.y = pos_src[1]
        msg_luz.pose.position.z = pos_src[2]
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
            
            # Si la cámara o la fuente de luz están "detrás" del panel, ignoramos
            if cam_local[2] <= 0 or src_local[2] <= 0:
                continue
            
            ref_local = np.array([src_local[0], src_local[1], -src_local[2]])
            denominador = cam_local[2] - ref_local[2]
            
            # Evitamos divisiones por cero
            if abs(denominador) < 1e-6: 
                continue 
            
            t = -ref_local[2] / denominador
            i_local = ref_local + t * (cam_local - ref_local)
            
            # Comprobamos si el impacto cae dentro de los límites físicos del panel
            if abs(i_local[0]) <= (width / 2.0) and abs(i_local[1]) <= (length / 2.0): 
                i_world = pos_panel + rot_panel.apply(i_local)
                ref_world = pos_panel + rot_panel.apply(ref_local)
                
                # Estos se siguen publicando en global para Rviz
                pose_rebote = Pose()
                pose_rebote.position.x = i_world[0]
                pose_rebote.position.y = i_world[1]
                pose_rebote.position.z = i_world[2]
                msg_rebotes.poses.append(pose_rebote)
                
                pose_reflejo = Pose()
                pose_reflejo.position.x = ref_world[0]
                pose_reflejo.position.y = ref_world[1]
                pose_reflejo.position.z = ref_world[2]
                msg_reflejos.poses.append(pose_reflejo)
                
                normal_teorica = rot_panel.apply([0.0, 0.0, 1.0])
                
                # Formateo estructurado del diccionario JSON
                dato_impacto = {
                    "id_panel": id_panel,
                    "rebote_local": [
                        float(i_local[0]), 
                        float(i_local[1]), 
                        float(i_local[2])
                    ],
                    "normal_teorica": [
                        float(normal_teorica[0]), 
                        float(normal_teorica[1]), 
                        float(normal_teorica[2])
                    ],
                    "pose_panel": {
                        "pos": [
                            float(pos_panel[0]), 
                            float(pos_panel[1]), 
                            float(pos_panel[2])
                        ],
                        "quat": [
                            float(rot_panel.as_quat()[0]), 
                            float(rot_panel.as_quat()[1]), 
                            float(rot_panel.as_quat()[2]), 
                            float(rot_panel.as_quat()[3])
                        ]
                    },
                    "dron": {
                        "pos": [float(x_gz), float(y_gz), float(z_gz)],
                        "quat": [float(qx_gz), float(qy_gz), float(qz_gz), float(qw_gz)]
                    }
                }
                lista_datos_consolidados.append(dato_impacto)
                
        self.pub_rebotes.publish(msg_rebotes)
        self.pub_reflejos.publish(msg_reflejos)
        
        if lista_datos_consolidados:
            msg_json = String()
            msg_json.data = json.dumps(lista_datos_consolidados)
            self.pub_datos_consolidados.publish(msg_json)


def main(args=None):
    rclpy.init(args=args)
    nodo = OpticsCalculatorNode(nombre_mundo="prueba1", modelo_dron="x500")
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
