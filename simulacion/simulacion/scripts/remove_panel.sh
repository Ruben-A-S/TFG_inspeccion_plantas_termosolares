#!/bin/bash

# Comprobación de argumentos
if [ "$#" -ne 2 ]; then
  echo -e "\e[31m[ERROR] Sintaxis incorrecta.\e[0m"
  echo "Uso correcto: ./remove_panel.sh <world> <nombre>"
  echo "Ejemplo:      ./remove_panel.sh mi_mundo obstaculo_2"
  exit 1
fi

WORLD="$1"
NOMBRE="$2"

# Comando para eliminar la entidad en Gazebo (type: 2 hace referencia a un 'MODEL')
gz service -s "/world/$WORLD/remove" \
  --reqtype gz.msgs.Entity \
  --reptype gz.msgs.Boolean \
  --timeout 2000 \
  --req "name: \"$NOMBRE\", type: 2"

# Comprobamos el código de salida del comando de Gazebo
if [ $? -eq 0 ]; then
  # Si todo ha ido bien, salimos con éxito. 
  # Python capturará este exit 0 y sabrá que funcionó.
  exit 0
else
  # Si falla, mandamos el error a la salida de errores estándar (>&2)
  echo -e "\e[31m[ERROR] Fallo interno de Gazebo al intentar eliminar '$NOMBRE'.\e[0m" >&2
  exit 1
fi
