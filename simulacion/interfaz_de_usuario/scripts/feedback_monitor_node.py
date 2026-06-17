#!/usr/bin/env python3

import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Definimos el perfil de comunicación EXACTO para evitar el conflicto de QoS
mi_qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=10
)

class Color:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'

class FeedbackMonitorNode(Node):
    """
    Nodo subscriptor de diferentes tópicos que ofrece 
    feedback del estado de la simulación.

    Además de mostrar feedback de las órdenes enviadas con la interfaz, 
    permite visualizar resultados del método de inspección, 
    mostrando el error de orientación medido.
    """
    def __init__(self):
        super().__init__('feedback_monitor_node')
        
        self.ultima_impresion = {} 
        self.intervalo_refresco = 1.0 

        # --- SUSCRIPCIONES ---
        self.create_subscription(String, '/sim_status/log', self.log_callback, 10)
        self.create_subscription(String, '/sim_status/state', self.estado_callback, 10)
        self.create_subscription(String, '/sim_status/panel_updates', self.updates_callback, 10)
        self.create_subscription(String, '/calibration/results', self.resultados_callback, 10)
        
        # Suscripción al Faker con el perfil de QoS forzado para evitar el WARNING
        self.create_subscription(String, '/inspection/raw_data', self.raw_data_callback, mi_qos_profile)
        
        self.estado_actual = ""

        print(f"\n{Color.OKCYAN}{Color.BOLD}" + "=" * 45)
        print("         MONITOR CENTRAL DE INSPECCIÓN TERMOSOLAR ACTIVO")
        print("=" * 45 + f"{Color.ENDC}\n")

    def estado_callback(self, msg):
        nuevo_estado = msg.data
        if nuevo_estado != self.estado_actual:
            self.estado_actual = nuevo_estado
            print(f"{Color.BOLD}{Color.OKBLUE}[SISTEMA]{Color.ENDC} Estado: {Color.BOLD}{nuevo_estado}{Color.ENDC}")

    def updates_callback(self, msg):
        try:
            ids = json.loads(msg.data)
            if ids:
                print(f"{Color.OKCYAN}[UPDATE]{Color.ENDC} Entidades modificadas: {ids}")
            else:
                print(f"{Color.WARNING}[UPDATE]{Color.ENDC} Mapa vaciado.")
        except:
            pass

    def raw_data_callback(self, msg):
        try:
            impactos = json.loads(msg.data)
            ahora = time.time()
            
            for imp in impactos:
                p_id = imp.get("id_panel", "Desconocido")
                llave_spam = f"{p_id}_raw"
                
                if ahora - self.ultima_impresion.get(llave_spam, 0.0) > self.intervalo_refresco:
                    self.ultima_impresion[llave_spam] = ahora
                    
                    rebote_w = imp.get("rebote_world_debug", [0, 0, 0])
                    rebote_l = imp.get("rebote_local", [0, 0, 0])
                    
                    tipo = "FACETA" if "_f" in p_id else "PANEL"
                    
                    print(f"{Color.WARNING}{Color.BOLD}[IMPACTO]{Color.ENDC} "
                          f"{tipo}: {Color.BOLD}{p_id}{Color.ENDC} | "
                          f"Global: ({rebote_w[0]:.1f}, {rebote_w[1]:.1f}, {rebote_w[2]:.1f})")
        except:
            pass

    def resultados_callback(self, msg):
        try:
            res = json.loads(msg.data)
            print(f"\n{Color.OKGREEN}{Color.BOLD}=== REPORTE DE CALIBRACIÓN HELIOPOINT ==={Color.ENDC}")
            
            for p in res:
                p_id = p.get('id', 'Desconocido')
                muestras = p.get('muestras_tomadas', 0)
                error_x = p.get('error_x_mrad', 0.0)
                error_y = p.get('error_y_mrad', 0.0)
                
                tipo = "FACETA" if "_f" in p_id else "PANEL"
                print(f"{Color.OKGREEN}{Color.BOLD}[{tipo} {p_id}]{Color.ENDC} Muestras: {muestras}")
                print(f" ├─ Error Media -> X: {error_x:.2f} mrad | Y: {error_y:.2f} mrad")
            
            print(f"{Color.OKGREEN}{Color.BOLD}========================================={Color.ENDC}\n")
                
        except Exception as e:
            print(f"{Color.FAIL}[ERROR] Fallo al leer resultados de calibración: {e}{Color.ENDC}")
    
    def log_callback(self, msg):
        texto = msg.data
        if any(word in texto for word in ["ERROR", "FAIL"]):
            print(f"{Color.FAIL}{Color.BOLD}[¡ERROR!] {Color.ENDC}{Color.FAIL}{texto}{Color.ENDC}")
        elif any(word in texto.upper() for word in ["ÉXITO", "OK"]):
            print(f"{Color.OKGREEN}{Color.BOLD}[OK] {Color.ENDC}{texto}")

def main(args=None):
    rclpy.init(args=args)
    nodo = FeedbackMonitorNode()
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
