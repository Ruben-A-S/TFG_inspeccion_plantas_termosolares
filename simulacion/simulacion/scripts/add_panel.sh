#!/bin/bash

# Comprobación de argumentos
if [ "$#" -ne 7 ]; then
  echo -e "\e[31m[ERROR] Sintaxis incorrecta.\e[0m"
  echo "Uso correcto: ./add_panel.sh <world> <nombre> <x> <y> <z> <pitch> <yaw>"
  echo "Ejemplo:      ./add_panel.sh mi_mundo obstaculo_2 6.0 2.0 1.0 0.785 0.0"
  exit 1
fi

WORLD="$1"
NOMBRE="$2"
POS_X="$3"
POS_Y="$4"
POS_Z="$5"
PITCH="$6"
YAW="$7"

echo "Calculando orientación e inyectando panel '$NOMBRE'..."

# 1. OPTIMIZACIÓN: Calculamos todos los cuaterniones en una sola llamada a Python.
# Llamar a Python 4 veces en Bash es muy lento. Con 'read' asignamos las 4 variables de golpe.
read -r Q_X Q_Y Q_Z Q_W <<< $(python3 -c "
import math
qx = -math.sin($PITCH/2.0) * math.sin($YAW/2.0)
qy = math.sin($PITCH/2.0) * math.cos($YAW/2.0)
qz = math.cos($PITCH/2.0) * math.sin($YAW/2.0)
qw = math.cos($PITCH/2.0) * math.cos($YAW/2.0)
print(f'{qx} {qy} {qz} {qw}')
")

# 2. RUTAS DINÁMICAS: Usamos $HOME en lugar de hardcodear 'home/ruben' (al que le faltaba la '/' inicial)
RUTA_MODELO="$HOME/Carpeta_TFG_Provisional/src/TFG_inspeccion_plantas_termosolares/simulacion/simulation_tools/models/panel.sdf"

# 3. COMANDO GZ
gz service -s "/world/$WORLD/create" \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 2000 \
  --req "sdf_filename: \"$RUTA_MODELO\", name: \"$NOMBRE\", pose: {position: {x: $POS_X, y: $POS_Y, z: $POS_Z}, orientation: {x: $Q_X, y: $Q_Y, z: $Q_Z, w: $Q_W}}"

# Comprobamos si el comando anterior falló o tuvo éxito
if [ $? -eq 0 ]; then
  echo -e "\e[32m[ÉXITO] Panel '$NOMBRE' inyectado.\e[0m"
else
  echo -e "\e[31m[ERROR] Fallo al inyectar el panel.\e[0m"
fi
