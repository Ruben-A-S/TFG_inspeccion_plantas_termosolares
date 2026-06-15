#!/usr/bin/env python3

import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import numpy as np
from scipy.spatial.transform import Rotation as R

class RVizCalibrationMarkersNode(Node):
    """
    Nodo encargado de suscribirse a los resultados de HelioPoint
    y publicar MarkerArrays en RViz para visualizar los vectores normales,
    incluyendo la reconstrucción de la Normal Media Acumulada.
    Adaptado para soportar Heliostatos Multifacetados.
    """
    def __init__(self):
        super().__init__('rviz_calibration_markers_node')
        
        self.paneles_teoria = {}
        
        self.cli_teoria = self.create_client(Trigger, 'get_panel_theory')
        self.create_subscription(String, '/sim_status/panel_updates', self.actualizar_teoria_callback, 10)
        self.create_subscription(String, '/calibration/results', self.resultados_callback, 10)
        
        self.pub_markers = self.create_publisher(MarkerArray, '/calibration/rviz_markers', 10)
        
        self.frame_id = "world"  
        # NOTA: He reducido la longitud de la flecha de 5.0 a 3.0 para que 
        # en paneles de 25 facetas no se forme un borrón de colores gigante.
        self.longitud_flecha = 3.0  

        self.pedir_mapa_teorico()
        self.get_logger().info("Visualizador de Flechas RViz iniciado [MODO FACETAS]. Esperando vectores...")

    def pedir_mapa_teorico(self):
        if not self.cli_teoria.service_is_ready():
            return
        req = Trigger.Request()
        self.cli_teoria.call_async(req).add_done_callback(self.al_recibir_teoria)

    # ==========================================
    # LA MAGIA: APLANAR LA TEORÍA PARA LAS FLECHAS
    # ==========================================
    def al_recibir_teoria(self, futuro):
        try:
            res = futuro.result()
            if res.success:
                lista = json.loads(res.message)
                self.paneles_teoria = {}
                
                for p in lista:
                    r_track = R.from_euler('xyz', [0.0, p['pitch'], p['yaw']])
                    p_centro = np.array([p['x'], p['y'], p['z']])
                    
                    if 'facetas' in p:
                        for f in p['facetas']:
                            pos_f = p_centro + r_track.apply(f['offset'])
                            r_f = r_track * R.from_euler('xyz', [0.0, f['cant_pitch'], f['cant_yaw']])
                            euler_f = r_f.as_euler('xyz')
                            
                            self.paneles_teoria[str(f['id'])] = {
                                'x': pos_f[0], 'y': pos_f[1], 'z': pos_f[2],
                                'pitch': euler_f[1], 'yaw': euler_f[2]
                            }
                    else:
                        self.paneles_teoria[str(p['id'])] = {
                            'x': p['x'], 'y': p['y'], 'z': p['z'],
                            'pitch': p['pitch'], 'yaw': p['yaw']
                        }
                        
                self.get_logger().info(f"Puntos de anclaje de flechas listos: {len(self.paneles_teoria)} orígenes.")
        except Exception as e:
            self.get_logger().error(f"Error cargando teoría: {e}")

    def actualizar_teoria_callback(self, msg):
        self.pedir_mapa_teorico()

    def resultados_callback(self, msg):
        if not self.paneles_teoria:
            return 
            
        try:
            resultados = json.loads(msg.data)
            marker_array = MarkerArray()
            
            for p in resultados:
                p_id = str(p.get("id"))
                
                if p_id not in self.paneles_teoria:
                    continue
                    
                panel_teo = self.paneles_teoria[p_id]
                base_x = panel_teo['x']
                base_y = panel_teo['y']
                base_z = panel_teo['z']
                
                # 1. Extraer los vectores básicos
                n_teo = p.get("normal_teorica", [0.0, 0.0, 1.0])
                n_med = p.get("normal_medida", [0.0, 0.0, 1.0])
                
                # 2. Extraer los errores medios y pasarlos a radianes
                error_x_rad = p.get("error_x_mrad", 0.0) / 1000.0
                error_y_rad = p.get("error_y_mrad", 0.0) / 1000.0
                
                # 3. RECONSTRUIR LA NORMAL MEDIA
                pitch = panel_teo.get('pitch', 0.0)
                yaw = panel_teo.get('yaw', 0.0)
                r_teo = R.from_euler('xyz', [0.0, pitch, yaw])
                
                r_error = R.from_euler('xy', [error_x_rad, error_y_rad])
                n_media_ccs = r_error.apply([0.0, 0.0, 1.0])
                n_media_global = r_teo.apply(n_media_ccs)
                
                hash_id = abs(hash(str(p_id))) % 100000
                
                # Flecha Teórica (AZUL)
                marker_teo = self.crear_flecha(
                    id_marcador=hash_id,
                    x=base_x, y=base_y, z=base_z,
                    vector=n_teo,
                    ns="1_normal_teorica",
                    color=(0.0, 0.5, 1.0)
                )
                
                # Flecha Medida Instantánea (ROJA)
                marker_med = self.crear_flecha(
                    id_marcador=hash_id + 100000,
                    x=base_x, y=base_y, z=base_z,
                    vector=n_med,
                    ns="2_normal_medida_instantanea",
                    color=(1.0, 0.0, 0.2)
                )
                
                # Flecha Media Acumulada (VERDE)
                marker_media = self.crear_flecha(
                    id_marcador=hash_id + 200000,
                    x=base_x, y=base_y, z=base_z,
                    vector=n_media_global.tolist(),
                    ns="3_normal_media_acumulada",
                    color=(0.1, 0.9, 0.1) 
                )
                
                marker_array.markers.append(marker_teo)
                marker_array.markers.append(marker_med)
                marker_array.markers.append(marker_media)
                
            if marker_array.markers:
                self.pub_markers.publish(marker_array)
                
        except Exception as e:
            self.get_logger().error(f"Error al procesar resultados para RViz: {e}")

    def crear_flecha(self, id_marcador, x, y, z, vector, ns, color):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        
        marker.ns = ns
        marker.id = id_marcador
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        p_inicio = Point(x=float(x), y=float(y), z=float(z))
        p_fin = Point(
            x=float(x + (vector[0] * self.longitud_flecha)),
            y=float(y + (vector[1] * self.longitud_flecha)),
            z=float(z + (vector[2] * self.longitud_flecha))
        )
        marker.points = [p_inicio, p_fin]
        
        # Escala: grosor de la flecha
        marker.scale.x = 0.10  
        marker.scale.y = 0.20  
        marker.scale.z = 0.20  
        
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 0.9 
        
        marker.lifetime.sec = 2
        marker.lifetime.nanosec = 0
        
        return marker

def main(args=None):
    rclpy.init(args=args)
    nodo = RVizCalibrationMarkersNode()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            nodo.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
