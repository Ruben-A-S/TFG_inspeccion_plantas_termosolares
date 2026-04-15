# TFG_inspeccion_plantas_termosolares
Repositorio utilizado para el desarrollo de mi TFG. Cuyo objetivo es la simulación de inspección de plantas termosolares utilizando uno o varios UAV.

Este proyecto hace uso de PX4-Autopilot, que se puede instalar en la página oficial de este software:
[Aqui meteré el enlace en un futuro]


Dentro de la carpeta de simulacion encontramos otras tres carpetas.
Comenzamos con simulation_tools, en esta carpeta se encuentran herramientas útiles y utilizadas en los .launch de los archivos finales encontrados en las otras carpetas.

Dentro de simulation_tools, encontramos una carpeta para almacenar modelos y texturas, otra que almacena mundos y otra con los scripts.
Los scripts destacados son:
 - control_sim_node.py: este nodo se utiliza para la configuración del mundo, subscribiendose a topics de configuracion.
 - interfaz_terminal_node.py: este script actúa de interfaz por terminal para que el usuario pueda enviar cómodamente los topics necesarios para la configuración y generación del mundo.
