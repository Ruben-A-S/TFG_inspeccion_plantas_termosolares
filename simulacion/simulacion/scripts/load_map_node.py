import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import csv
import os
import numpy as np  

# Importamos tus scripts externos
from Add_panels_from_file import inyectar_paneles
from Remove_panels_from_file import eliminar_paneles


# ==========================================
# FUNCIONES MATEMÁTICAS EXTERNAS
# ==========================================
def obtener_sol_inventado(fecha, hora):
    """
    Sol de mentira (Mock). Devuelve unas coordenadas X, Y, Z estáticas.
    Cuando sepas cómo calcularlo, metes tu lógica aquí.
    """
    return [1000.0, 0.0, 500.0]

def calcular_orientacion_heliostato(pos_panel, pos_torre, posicion_sol):
    """
    Calcula los ángulos Yaw y Pitch para un heliostato usando Álgebra Vectorial.
    """
    # 1. Vector hacia el Sol (S)
    Vector_S = np.array(posicion_sol) - np.array(pos_panel)
    Vector_S = Vector_S / np.linalg.norm(Vector_S)
    
    # 2. Vector hacia la Torre (T)
    Vector_T = np.array(pos_torre) - np.array(pos_panel)
    Vector_T = Vector_T / np.linalg.norm(Vector_T)
    
    # 3. Vector Normal (N) (La bisectriz)
    Vector_N = Vector_S + Vector_T
    Vector_N = Vector_N / np.linalg.norm(Vector_N)
    
    # 4. Convertir a ángulos para Gazebo (Yaw, Pitch)
    yaw = np.arctan2(Vector_N[1], Vector_N[0])
    pitch = np.arcsin(Vector_N[2])
    
    return float(yaw), float(pitch)


# ==========================================
# CLASE DEL NODO ROS 2
# ==========================================
class LoadMapNode(Node):
    def __init__(self):
        super().__init__('load_map_node')

        self.array_paneles = []
        
        # --- SUBSCRIPTOR ---
        self.create_subscription(String, '/sim_cmd/gestion_mapa', self.cb_gestion_mapa, 10)

        # --- PUBLICADOR DE LOGS ---
        self.pub_log = self.create_publisher(String, '/sim_status/log', 10)

        # --- PUBLICADOR DEL ARRAY ---
        self.pub_paneles = self.create_publisher(String, '/sim_data/paneles_info', 100)

        # --- TEMPORIZADOR PARA PUBLICAR ARRAY ---
        self.timer = self.create_timer(2.0, self.timer_callback)

        self.enviar_log("Nodo Load Map iniciado. Esperando órdenes...")
        
    def timer_callback(self):
        """Publica el estado actual del mapa periódicamente"""
        msg = String()
        # Publicamos el array (si está vacío, enviará [], avisando que no hay nada)
        msg.data = json.dumps(self.array_paneles)
        self.pub_paneles.publish(msg)

    def cb_gestion_mapa(self, msg):
        try:
            datos = json.loads(msg.data)
            accion = datos.get("accion")
            mundo = datos.get("mundo")
            csv_file = datos.get("csv")
            
            # Extraemos fecha y hora por si hacen falta para el sol
            fecha = datos.get("fecha", "10/02/2001")
            hora = datos.get("hora", "12:00")

            if accion == "CARGAR":
                modelo = datos.get("modelo")
                self.ejecutar_carga(mundo, csv_file, modelo, fecha, hora)
            elif accion == "VACIAR":
                self.ejecutar_vaciado(mundo, csv_file)

        except json.JSONDecodeError:
            self.enviar_log("ERROR: Se recibió un JSON corrupto en gestion_mapa.")

    def ejecutar_carga(self, mundo, csv_file, modelo, fecha, hora):
        self.enviar_log(f"Iniciando cálculo de posiciones desde {csv_file}...")
        
        # 1. Leemos el CSV y calculamos los ángulos
        self.array_paneles = self.generar_array_desde_csv(csv_file, fecha, hora)
        
        # 2. Publicamos el JSON de los paneles para el resto de la simulación
        msg_array = String()
        msg_array.data = json.dumps(self.array_paneles)
        self.pub_paneles.publish(msg_array)
        self.enviar_log(f"Array con {len(self.array_paneles)} paneles publicado en /sim_data/paneles_info")
        
        # 3. Inyectamos los paneles físicos en Gazebo
        try:
            self.enviar_log(f"Inyectando {len(self.array_paneles)} paneles en Gazebo...")
            inyectar_paneles(mundo, self.array_paneles, modelo)
            self.enviar_log("Inyección completada con éxito.")
        except Exception as e:
            self.enviar_log(f"ERROR en script de inyección: {e}")

    def ejecutar_vaciado(self, mundo, csv_file):
        self.enviar_log("Iniciando proceso de vaciado de paneles...")
        
        # Generamos el array para saber qué borrar
        self.array_paneles = self.generar_array_desde_csv(csv_file, "00/00/0000", "00:00")
        
        try:
            eliminar_paneles(mundo, self.array_paneles)
            
            # Publicamos una lista vacía para avisar a los demás nodos
            msg_vacio = String()
            msg_vacio.data = json.dumps([]) 
            self.pub_paneles.publish(msg_vacio)
            
            self.enviar_log("Vaciado completado. Array vacío publicado.")
        except Exception as e:
            self.enviar_log(f"ERROR en script de vaciado: {e}")

    # ==========================================
    # CÁLCULOS DEL CSV
    # ==========================================
    def generar_array_desde_csv(self, nombre_csv, fecha, hora):
        ruta_absoluta = os.path.expanduser(f"~/{nombre_csv}")
        # CREAMOS UNA LISTA LOCAL (Sin el self.)
        lista_temporal = [] 
        
        try:
            with open(ruta_absoluta, mode='r', encoding='utf-8') as archivo:
                next(archivo)
                lector_csv = csv.DictReader(archivo)
                
                for fila in lector_csv:
                    if len(lista_temporal) >= 5:
                        break
                    
                    x, y, z = float(fila["Heliostat x"]), float(fila["Heliostat y"]), float(fila["Heliostat z"])
                    aim_x, aim_y, aim_z = float(fila["Aiming point x"]), float(fila["Aiming point y"]), float(fila["Aiming point z"])
                    
                    pos_sol = obtener_sol_inventado(fecha, hora)
                    yaw, pitch = calcular_orientacion_heliostato([x,y,z], [aim_x,aim_y,aim_z], pos_sol)

                    panel = {
                        "id": f"panel_{len(lista_temporal) + 1}", # Usamos lista_temporal
                        "x": x, "y": y, "z": z,
                        "f_len_x": float(fila["Focal Length x"]),
                        "f_len_y": float(fila["Focal Length y"]),
                        "aim_x": aim_x, "aim_y": aim_y, "aim_z": aim_z,
                        "width_x": float(fila["Heliostat width (x)"]),
                        "length_y": float(fila["Heliostat length (y)"]),
                        "yaw": yaw,
                        "pitch": pitch
                    }
                    lista_temporal.append(panel) # Añadimos a la local
                    
            self.enviar_log(f"CSV leído. Se prepararon {len(lista_temporal)} paneles.")
            return lista_temporal # DEVOLVEMOS LA LISTA COMPLETA AL FINAL
            
        except Exception as e:
            self.enviar_log(f"ERROR: {e}")
            return []
    def enviar_log(self, texto):
        msg = String()
        msg.data = f"[LOAD_MAP] {texto}"
        self.pub_log.publish(msg)
        self.get_logger().info(texto)

def main(args=None):
    rclpy.init(args=args)
    nodo = LoadMapNode()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
