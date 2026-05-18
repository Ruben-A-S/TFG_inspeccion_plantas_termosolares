#!/usr/bin/env python3

import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

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
    def __init__(self):
        super().__init__('feedback_monitor_node')
        
        # --- CONTROL DE SPAM (Rate Limiting) ---
        self.ultima_impresion = {} # Diccionario para guardar cuándo se imprimió por última vez algo de un panel
        self.intervalo_refresco = 1.0 # Segundos de espera antes de volver a imprimir datos del MISMO panel

        # --- SUSCRIPCIONES ---
        self.create_subscription(String, '/sim_status/log', self.log_callback, 10)
        self.create_subscription(String, '/sim_status/state', self.estado_callback, 10)
        self.create_subscription(String, '/sim_status/panel_updates', self.updates_callback, 10)
        
        # Suscripción al Cerebro (Calibración Heliopoint)
        self.create_subscription(String, '/calibration/results', self.resultados_callback, 10)
        
        # Suscripción al Faker (Para ver dónde está rebotando físicamente la luz)
        self.create_subscription(String, '/inspection/raw_data', self.raw_data_callback, 10)
        
        self.estado_actual = ""

        print(f"\n{Color.OKCYAN}{Color.BOLD}" + "=" * 65)
        print("         MONITOR CENTRAL DE INSPECCIÓN TERMOSOLAR ACTIVO")
        print("=" * 65 + f"{Color.ENDC}\n")

    def estado_callback(self, msg):
        nuevo_estado = msg.data
        if nuevo_estado != self.estado_actual:
            self.estado_actual = nuevo_estado
            print(f"{Color.BOLD}{Color.OKBLUE}[SISTEMA]{Color.ENDC} Estado: {Color.BOLD}{nuevo_estado}{Color.ENDC}")

    def updates_callback(self, msg):
        try:
            ids = json.loads(msg.data)
            if ids:
                print(f"{Color.OKCYAN}[UPDATE]{Color.ENDC} Paneles modificados: {ids}")
            else:
                print(f"{Color.WARNING}[UPDATE]{Color.ENDC} Mapa vaciado.")
        except:
            pass

    def raw_data_callback(self, msg):
        """Muestra la posición física donde el rayo del dron está golpeando el espejo."""
        try:
            impactos = json.loads(msg.data)
            ahora = time.time()
            
            for imp in impactos:
                p_id = imp.get("id_panel", "Desconocido")
                llave_spam = f"{p_id}_raw"
                
                # Limitador: Si no ha pasado 1 segundo, ignoramos el print
                if ahora - self.ultima_impresion.get(llave_spam, 0.0) > self.intervalo_refresco:
                    self.ultima_impresion[llave_spam] = ahora
                    
                    rebote_w = imp.get("rebote_world_debug", [0, 0, 0])
                    rebote_l = imp.get("rebote_local", [0, 0, 0])
                    dist = imp.get("distancia", 0.0)
                    
                    print(f"{Color.WARNING}{Color.BOLD}[IMPACTO]{Color.ENDC} "
                          f"Panel: {Color.BOLD}{p_id}{Color.ENDC} | "
                          f"Dist: {dist:.1f}m | "
                          f"Global(x,y,z): ({rebote_w[0]:.1f}, {rebote_w[1]:.1f}, {rebote_w[2]:.1f}) | "
                          f"Local: ({rebote_l[0]:.2f}, {rebote_l[1]:.2f})")
        except:
            pass

    def resultados_callback(self, msg):
        """Muestra todos los errores y vectores calculados por el cerebro sin filtrar."""
        try:
            res = json.loads(msg.data)
            
            # Encabezado visual para separar lotes de cálculo
            print(f"\n{Color.OKGREEN}{Color.BOLD}=== REPORTE DE CALIBRACIÓN HELIOPOINT ==={Color.ENDC}")
            
            for p in res:
                p_id = p.get('id', 'Desconocido')
                muestras = p.get('muestras_tomadas', 0)
                
                # Extraemos errores (mostramos las medias, que son más representativas)
                error_x = p.get('error_x_mrad', 0.0)
                error_y = p.get('error_y_mrad', 0.0)
                
                # Extraemos vectores
                n_teo = p.get('normal_teorica', [0.0, 0.0, 0.0])
                n_med = p.get('normal_medida', [0.0, 0.0, 0.0])
                
                # Formateamos los vectores a 3 decimales para que no ensucien la terminal
                n_teo_str = f"({n_teo[0]:.3f}, {n_teo[1]:.3f}, {n_teo[2]:.3f})"
                n_med_str = f"({n_med[0]:.3f}, {n_med[1]:.3f}, {n_med[2]:.3f})"
                
                # Imprimimos toda la información del panel
                print(f"{Color.OKGREEN}{Color.BOLD}[PANEL {p_id}]{Color.ENDC} Muestras procesadas: {muestras}")
                print(f" ├─ Desviación Media -> Error X: {error_x:.2f} mrad | Error Y: {error_y:.2f} mrad")
                print(f" └─ Vectores Globales -> Teórico: {n_teo_str} | Medido: {n_med_str}")
            
            print(f"{Color.OKGREEN}{Color.BOLD}========================================={Color.ENDC}\n")
                
        except Exception as e:
            print(f"{Color.FAIL}[ERROR] Fallo al leer resultados de calibración: {e}{Color.ENDC}")
    
    
    def log_callback(self, msg):
        texto = msg.data
        if any(word in texto for word in ["ERROR", "FAIL", "incorrecto"]):
            print(f"{Color.FAIL}{Color.BOLD}[¡ERROR!] {Color.ENDC}{Color.FAIL}{texto}{Color.ENDC}")
        elif any(word in texto.upper() for word in ["ÉXITO", "COMPLETADO", "OK"]):
            print(f"{Color.OKGREEN}{Color.BOLD}[OK] {Color.ENDC}{texto}")
        elif "DESTELLO" in texto or "DETECTADO" in texto:
            # Los destellos de la cámara los pasamos en silencio ahora, ya que los gestiona raw_data
            pass 
        #else:
            #print(f"  • {texto}")

def main(args=None):
    rclpy.init(args=args)
    nodo = FeedbackMonitorNode()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        print(f"\n{Color.WARNING}Cerrando monitor...{Color.ENDC}")
    finally:
        if rclpy.ok():
            nodo.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
