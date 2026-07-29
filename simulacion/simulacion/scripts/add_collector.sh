#!/bin/bash

# Argument check
if [ "$#" -ne 7 ]; then
  echo -e "\e[31m[ERROR] Incorrect syntax.\e[0m"
  echo "Correct usage: ./add_collector.sh <world> <name> <x> <y> <z> <pitch> <yaw>"
  echo "Example:       ./add_collector.sh my_world obstacle_2 6.0 2.0 1.0 0.785 0.0"
  exit 1
fi

WORLD="$1"
NAME="$2"
POS_X="$3"
POS_Y="$4"
POS_Z="$5"
PITCH="$6"
YAW="$7"

echo "Calculating orientation and injecting collector '$NAME'..."

# 1. OPTIMIZATION: We calculate all quaternions in a single Python call.
# Calling Python 4 times in Bash is very slow. With 'read' we assign all 4 variables at once.
read -r Q_X Q_Y Q_Z Q_W <<< $(python3 -c "
import math
qx = -math.sin($PITCH/2.0) * math.sin($YAW/2.0)
qy = math.sin($PITCH/2.0) * math.cos($YAW/2.0)
qz = math.cos($PITCH/2.0) * math.sin($YAW/2.0)
qw = math.cos($PITCH/2.0) * math.cos($YAW/2.0)
print(f'{qx} {qy} {qz} {qw}')
")

# 2. DYNAMIC PATHS: We use $HOME instead of hardcoding 'home/ruben' (which was missing the leading '/')
MODEL_PATH="$HOME/Carpeta_TFG_Provisional/src/TFG_inspeccion_plantas_termosolares/simulacion/simulation_tools/models/collector.sdf"

# 3. GZ COMMAND
gz service -s "/world/$WORLD/create" \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 2000 \
  --req "sdf_filename: \"$MODEL_PATH\", name: \"$NAME\", pose: {position: {x: $POS_X, y: $POS_Y, z: $POS_Z}, orientation: {x: $Q_X, y: $Q_Y, z: $Q_Z, w: $Q_W}}"

# Check if the previous command failed or succeeded
if [ $? -eq 0 ]; then
  echo -e "\e[32m[SUCCESS] Collector '$NAME' injected.\e[0m"
else
  echo -e "\e[31m[ERROR] Failed to inject collector.\e[0m"
fi
