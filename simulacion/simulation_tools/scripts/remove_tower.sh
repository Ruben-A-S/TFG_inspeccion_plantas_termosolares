#!/bin/bash

if [ "$#" -ne 2 ]; then
  echo "Error de sintaxis."
  echo "Uso correcto: ./remove_panel.sh <world> <nombre>"
  echo "Ejemplo: ./remove_panel.sh mi_mundo obstaculo_2"
  exit 1
fi

WORLD=$1
NOMBRE=$2

gz service -s /world/$WORLD/remove \
--reqtype gz.msgs.Entity \
--reptype gz.msgs.Boolean \
--timeout 2000 \
--req "name: \"$NOMBRE\", type: 2"
