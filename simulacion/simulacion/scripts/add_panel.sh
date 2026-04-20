#!/bin/bash

if [ "$#" -ne 7 ]; then
  echo "Error de sintaxis."
  echo "Uso correcto: ./add_panel.sh <world> <nombre> <x> <y> <z> <pitch> <yaw>"
  echo "Ejemplo: ./add_panel.sh mi_mundo obstaculo_2 6.0 2.0 1.0 0.785 0.0"
  exit 1
fi

WORLD=$1
NOMBRE=$2
POS_X=$3
POS_Y=$4
POS_Z=$5
PITCH=$6
YAW=$7

Q_X=$(python3 -c "import math; print(-math.sin($PITCH/2.0) * math.sin($YAW/2.0))")
Q_Y=$(python3 -c "import math; print(math.sin($PITCH/2.0) * math.cos($YAW/2.0))")
Q_Z=$(python3 -c "import math; print(math.cos($PITCH/2.0) * math.sin($YAW/2.0))")
Q_W=$(python3 -c "import math; print(math.cos($PITCH/2.0) * math.cos($YAW/2.0))")

gz service -s /world/$WORLD/create \
--reqtype gz.msgs.EntityFactory \
--reptype gz.msgs.Boolean \
--timeout 2000 \
--req "sdf_filename: \"home/ruben/Carpeta_TFG_Provisional/src/TFG_inspeccion_plantas_termosolares/simulacion/simulation_tools/models/panel.sdf\", name: \"$NOMBRE\", pose: {position: {x: $POS_X, y: $POS_Y, z: $POS_Z}, orientation: {x: $Q_X, y: $Q_Y, z: $Q_Z, w: $Q_W}}"
