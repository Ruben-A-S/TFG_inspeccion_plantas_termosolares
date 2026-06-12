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
import matplotlib.patches as patches
from scipy.interpolate import griddata

class PanelAnalysisLoggerNode(Node):
    def __init__(self):
        super().__init__('panel_analysis_logger_node')
        
        # Geometría real del espejo
        self.ancho_fisico_m = 10.4  
        self.alto_fisico_m = 11.4   
        self.resolucion_malla = 50  
        
        self.csv_filename = 'historial_inspeccion_paneles.csv'
        file_exists = os.path.isfile(self.csv_filename)
        self.csv_file = open(self.csv_filename, mode='a', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        if not file_exists:
            self.csv_writer.writerow([
                'Timestamp', 'Panel_ID', 'Rebote_H', 'Rebote_V',
                'Inst_Error_rotX_mrad', 'Inst_Error_rotY_mrad'
            ])
            
        self.bridge = CvBridge()
        self.mallas_paneles = {}  
        self.panel_activo = None 
        
        self.create_subscription(String, '/calibration/results', self.resultados_callback, 10)
        self.pub_heatmap = self.create_publisher(Image, '/calibration/heatmap_image', 10)
        
        self.timer_heatmap = self.create_timer(2.5, self.publicar_mapa_calor)
        self.get_logger().info("Logger DLR 3D iniciado. Arquitectura de reconstrucción por Mínimos Cuadrados Vectorizada.")

    def resultados_callback(self, msg):
        try:
            resultados = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        timestamp_str = str(self.get_clock().now().to_msg().sec)

        for dato in resultados:
            p_id = dato.get("id")
            rebote = dato.get("rebote_local")
            
            if p_id is None or rebote is None:
                continue

            self.panel_activo = p_id

            err_x = dato.get("error_actual_rotX_mrad", 0.0)
            err_y = dato.get("error_actual_rotY_mrad", 0.0)
            
            coord_h = rebote[0] 
            coord_v = rebote[1] 
            
            self.csv_writer.writerow([timestamp_str, p_id, coord_h, coord_v, err_x, err_y])
            
            # --- AGRUPACIÓN DE DATOS CRUDOS ---
            if p_id not in self.mallas_paneles:
                self.mallas_paneles[p_id] = {
                    'suma_err_x': np.zeros((self.resolucion_malla, self.resolucion_malla)),
                    'suma_err_y': np.zeros((self.resolucion_malla, self.resolucion_malla)),
                    'conteo': np.zeros((self.resolucion_malla, self.resolucion_malla))
                }
                
            col_idx = int((coord_h + self.ancho_fisico_m / 2.0) / self.ancho_fisico_m * self.resolucion_malla)
            row_idx = int((coord_v + self.alto_fisico_m / 2.0) / self.alto_fisico_m * self.resolucion_malla)
            
            if 0 <= col_idx < self.resolucion_malla and 0 <= row_idx < self.resolucion_malla:
                self.mallas_paneles[p_id]['suma_err_x'][row_idx, col_idx] += err_x
                self.mallas_paneles[p_id]['suma_err_y'][row_idx, col_idx] += err_y
                self.mallas_paneles[p_id]['conteo'][row_idx, col_idx] += 1
                
        self.csv_file.flush()

    def integrar_superficie_vectorizada(self, slope_x, slope_y, dx, dy):
        """
        Integración acumulativa desde el centro usando np.cumsum.
        Elimina los bucles for manuales para mayor velocidad y limpieza matemática.
        """
        Z = np.zeros_like(slope_x)
        mid_r, mid_c = Z.shape[0] // 2, Z.shape[1] // 2

        # 1. Integrar la columna central (Eje Y)
        if mid_r + 1 < Z.shape[0]: # Hacia abajo
            Z[mid_r+1:, mid_c] = np.cumsum(slope_y[mid_r+1:, mid_c] * dy)
        if mid_r > 0:              # Hacia arriba
            Z[mid_r-1::-1, mid_c] = np.cumsum(-slope_y[mid_r-1::-1, mid_c] * dy)

        # 2. Integrar todas las filas a partir de la columna central (Eje X)
        if mid_c + 1 < Z.shape[1]: # Hacia la derecha
            Z[:, mid_c+1:] = Z[:, mid_c:mid_c+1] + np.cumsum(slope_x[:, mid_c+1:] * dx, axis=1)
        if mid_c > 0:              # Hacia la izquierda
            Z[:, mid_c-1::-1] = Z[:, mid_c:mid_c+1] + np.cumsum(-slope_x[:, mid_c-1::-1] * dx, axis=1)

        # Ajuste de gravedad (media = 0)
        return Z - np.mean(Z)

    def publicar_mapa_calor(self):
        fig, ax = plt.subplots(figsize=(8, 9)) 
        
        lim_h = self.ancho_fisico_m / 2.0
        lim_v = self.alto_fisico_m / 2.0
        
        ax.set_xlim(-lim_h - 1.0, lim_h + 1.0)
        ax.set_ylim(-lim_v - 1.0, lim_v + 1.0)
        ax.set_aspect('equal', 'box')
        ax.grid(True, linestyle='--', alpha=0.3)
        ax.set_xlabel('Ancho del Heliostato X (m)')
        ax.set_ylabel('Alto del Heliostato Y (m)')

        rect_panel = patches.Rectangle(
            (-lim_h, -lim_v), self.ancho_fisico_m, self.alto_fisico_m, 
            linewidth=4, edgecolor='black', facecolor='whitesmoke', linestyle='solid', zorder=1
        )
        ax.add_patch(rect_panel)

        if self.panel_activo is None or self.panel_activo not in self.mallas_paneles:
            ax.set_title("DLR Sim - Esperando escaneo del UAV...", fontsize=14)
        else:
            ax.set_title(f'Topografía 3D Reconstruida - Heliostato {self.panel_activo}', fontsize=12)
            
            datos_malla = self.mallas_paneles[self.panel_activo]
            conteo = datos_malla['conteo']
            mask = conteo > 0
            
            if np.sum(mask) >= 4:
                # =========================================================
                # PASO 1: EXTRAER DATOS CONOCIDOS
                # =========================================================
                err_x_crudo = datos_malla['suma_err_x'][mask] / conteo[mask]
                err_y_crudo = datos_malla['suma_err_y'][mask] / conteo[mask]
                
                # =========================================================
                # PASO 2: ASUMIR ORIENTACIÓN MEDIA Y CALCULAR DESVIACIÓN
                # =========================================================
                media_panel_x = np.mean(err_x_crudo)
                media_panel_y = np.mean(err_y_crudo)
                
                dev_x = err_x_crudo - media_panel_x
                dev_y = err_y_crudo - media_panel_y
                
                # Coordenadas de los puntos que sí hemos medido
                x_lin = np.linspace(-lim_h, lim_h, self.resolucion_malla)
                y_lin = np.linspace(-lim_v, lim_v, self.resolucion_malla)
                X_grid, Y_grid = np.meshgrid(x_lin, y_lin)
                pts_h = X_grid[mask]
                pts_v = Y_grid[mask]

                # =========================================================
                # PASO 3: INTERPOLAR PARA RECONSTRUIR PUNTOS NO MEDIDOS
                # =========================================================
                res_alta = 150
                xi = np.linspace(-lim_h, lim_h, res_alta)
                yi = np.linspace(-lim_v, lim_v, res_alta)
                XI, YI = np.meshgrid(xi, yi)
                
                try:
                    # Crear una "sábana" continua de pendientes para X e Y
                    slope_h_continuo = griddata((pts_h, pts_v), dev_x, (XI, YI), method='cubic')
                    slope_v_continuo = griddata((pts_h, pts_v), dev_y, (XI, YI), method='cubic')
                    
                    # Rellenar los huecos (bordes) donde la función cúbica no llega
                    slope_h_continuo[np.isnan(slope_h_continuo)] = griddata((pts_h, pts_v), dev_x, (XI, YI), method='nearest')[np.isnan(slope_h_continuo)]
                    slope_v_continuo[np.isnan(slope_v_continuo)] = griddata((pts_h, pts_v), dev_y, (XI, YI), method='nearest')[np.isnan(slope_v_continuo)]

                    # =========================================================
                    # PASO 4: RECONSTRUCCIÓN 3D (INTEGRACIÓN)
                    # =========================================================
                    dx_m = (lim_h * 2) / res_alta
                    dy_m = (lim_v * 2) / res_alta
                    
                    # Transformamos el mapa de ángulos en un mapa físico (mm)
                    Z = self.integrar_superficie_vectorizada(slope_h_continuo, slope_v_continuo, dx_m, dy_m)

                    # Recortar por fuera del cristal
                    fuera_espejo = (np.abs(XI) > lim_h) | (np.abs(YI) > lim_v)
                    Z[fuera_espejo] = np.nan

                    # --- RENDERIZADO VISUAL ---
                    max_z = np.nanmax(np.abs(Z))
                    if max_z < 0.5: max_z = 0.5

                    niveles = np.linspace(-max_z, max_z, 21) 
                    heatmap = ax.contourf(XI, YI, Z, levels=niveles, cmap='coolwarm', extend='both', alpha=0.9, zorder=2)
                    cbar = fig.colorbar(heatmap, fraction=0.046, pad=0.04)
                    cbar.set_label('Deformación de Altura (Milímetros)')
                    
                    texto_deformacion = f"Deformación Max:\n±{max_z:.2f} mm"
                    ax.text(lim_h + 0.1, -lim_v, texto_deformacion, fontsize=10, 
                            bbox=dict(facecolor='white', alpha=0.8, edgecolor='blue'), zorder=4)

                except Exception as e:
                    self.get_logger().error(f"Error topografía: {e}")

                # Cuadro de Tracking y Ruta
                texto_global = f"Error de Tracking:\nX: {media_panel_x:.1f} mrad\nY: {media_panel_y:.1f} mrad"
                ax.text(lim_h + 0.1, lim_v + 0.5, texto_global, fontsize=10, 
                        bbox=dict(facecolor='white', alpha=0.8, edgecolor='red'), zorder=4)

                ax.scatter(pts_h, pts_v, c='black', s=8, alpha=0.4, zorder=3, label='Ruta UAV')
                ax.legend(loc='upper left')

        fig.tight_layout()
        fig.canvas.draw()
        width, height = fig.canvas.get_width_height()
        imagen_rgb = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8).reshape(height, width, 3)
        plt.close(fig) 
        
        msg_img = self.bridge.cv2_to_imgmsg(imagen_rgb, encoding="rgb8")
        self.pub_heatmap.publish(msg_img)

def main(args=None):
    rclpy.init(args=args)
    nodo = PanelAnalysisLoggerNode()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.csv_file.close()
        if rclpy.ok():
            nodo.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
