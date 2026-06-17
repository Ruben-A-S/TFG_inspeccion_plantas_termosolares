#!/usr/bin/env python3

import os
import csv
import json
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

class PanelAnalysisLoggerNode(Node):
    """
    Dashboard de Inspección Multifaceta (Modo Tiempo Real).
    Muestra el estado actual de las facetas con una ventana deslizante implícita
    al recibir los datos procesados del CalibrationNode.
    """
    def __init__(self):
        super().__init__('panel_analysis_logger_node')
        
        self.filas = 5
        self.cols = 5
        
        self.csv_filename = 'historial_inspeccion_paneles.csv'
        file_exists = os.path.isfile(self.csv_filename)
        self.csv_file = open(self.csv_filename, mode='a', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        if not file_exists:
            self.csv_writer.writerow(['Timestamp', 'Panel_ID', 'Faceta_ID', 'Err_X_mrad', 'Err_Y_mrad'])
            
        self.bridge = CvBridge()
        
        # Diccionario principal de estado actual: { "panel_0": { "visitas", "err_x", "err_y" } }
        self.datos_planta = {}  
        self.panel_activo = None 
        
        self.create_subscription(String, '/calibration/results', self.resultados_callback, 10)
        self.pub_dashboard = self.create_publisher(Image, '/calibration/heatmap_image', 10)
        
        self.timer_dashboard = self.create_timer(1.5, self.publicar_dashboard)
        self.get_logger().info("Logger en tiempo real iniciado. Dashboard 5x5 refrescando...")

    def extraer_indices_faceta(self, id_faceta):
        try:
            partes = id_faceta.split('_f')
            if len(partes) != 2: return id_faceta, None, None
            
            id_padre = partes[0]
            indices = partes[1].split('_')
            return id_padre, int(indices[1]), int(indices[0])
        except:
            return id_faceta, None, None

    def resultados_callback(self, msg):
        try:
            resultados = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        timestamp = str(self.get_clock().now().to_msg().sec)

        for dato in resultados:
            id_completo = dato.get("id")
            if not id_completo: continue

            id_padre, c, f = self.extraer_indices_faceta(id_completo)
            if c is None or f is None: continue

            self.panel_activo = id_padre
            if id_padre not in self.datos_planta:
                self.datos_planta[id_padre] = {
                    'visitas': np.zeros((self.filas, self.cols)),
                    'err_x': np.zeros((self.filas, self.cols)),
                    'err_y': np.zeros((self.filas, self.cols))
                }

            # Actualización instantánea (Sobrescribimos el valor más reciente)
            err_x = dato.get("error_x_mrad", 0.0)
            err_y = dato.get("error_y_mrad", 0.0)
            
            self.datos_planta[id_padre]['err_x'][f, c] = err_x
            self.datos_planta[id_padre]['err_y'][f, c] = err_y
            self.datos_planta[id_padre]['visitas'][f, c] = 1 # Marcamos como activo
            
            self.csv_writer.writerow([timestamp, id_padre, id_completo, err_x, err_y])
        self.csv_file.flush()

    def publicar_dashboard(self):
        if self.panel_activo is None or self.panel_activo not in self.datos_planta:
            return

        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
        fig.suptitle(f'Dashboard Real-Time - {self.panel_activo}', fontsize=16, fontweight='bold')
        
        datos = self.datos_planta[self.panel_activo]
        mask = datos['visitas'] > 0

        for ax in [ax1, ax2]:
            ax.set_xticks(np.arange(self.cols + 1) - 0.5, minor=True)
            ax.set_yticks(np.arange(self.filas + 1) - 0.5, minor=True)
            ax.grid(which="minor", color="black", linestyle='-', linewidth=2)
            ax.tick_params(which="minor", size=0)
            ax.invert_yaxis()
            ax.set_aspect('equal')

        # 1. Cobertura
        ax1.set_title("Cobertura (Inspeccionado)", pad=15)
        ax1.imshow(datos['visitas'], cmap='Greens', origin='upper', vmin=0, vmax=1)

        # 2. Mapa Vectorial
        ax2.set_title("Desviación Normal (Canting Error)", pad=15)
        ax2.imshow(np.zeros((self.filas, self.cols)), cmap='gray', alpha=0.1, origin='upper')

        X, Y = np.meshgrid(np.arange(self.cols), np.arange(self.filas))
        U = np.where(mask, datos['err_x'], 0)
        V = np.where(mask, datos['err_y'], 0)
        mag = np.sqrt(U**2 + V**2)
        
        ax2.scatter(X[mask], Y[mask], color='black', s=10)
        if np.max(mag) > 0.01:
            q = ax2.quiver(X[mask], Y[mask], U[mask], V[mask], mag[mask], cmap='Reds', pivot='mid')
            fig.colorbar(q, ax=ax2, label='Magnitud Error (mrad)')

        fig.tight_layout()
        fig.canvas.draw()
        img = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8).reshape(fig.canvas.get_width_height()[::-1] + (3,))
        plt.close(fig) 
        
        self.pub_dashboard.publish(self.bridge.cv2_to_imgmsg(img, encoding="rgb8"))

def main(args=None):
    rclpy.init(args=args)
    nodo = PanelAnalysisLoggerNode()
    try: rclpy.spin(nodo)
    except KeyboardInterrupt: pass
    finally:
        nodo.csv_file.close()
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
